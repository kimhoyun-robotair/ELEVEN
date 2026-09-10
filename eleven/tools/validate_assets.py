#!/usr/bin/env python3
"""Validate delivered USD data and run elevator integration against real stages.

This script opens the composed scenes with OpenUSD. It checks geometry topology,
asset resolution, physics authoring, semantic controls and the bound controller.
It does not claim to run Isaac Sim/PhysX/RTX, for which a supported GPU and
Isaac Sim 5.1.0 installation are required.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys

from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def finite_vector(value):
    return all(math.isfinite(float(x)) for x in value)


def world_bounds(cache, prim):
    box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return ([float(v) for v in box.GetMin()], [float(v) for v in box.GetMax()])


def validate_geometry(stage, errors):
    counts = {"prims": 0, "meshes": 0, "vertices": 0, "polygons": 0,
              "materials": 0, "colliders": 0, "kinematic_bodies": 0}
    for prim in stage.Traverse():
        counts["prims"] += 1
        path = str(prim.GetPath())
        if prim.IsA(UsdShade.Material):
            counts["materials"] += 1
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            counts["colliders"] += 1
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            if UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get():
                counts["kinematic_bodies"] += 1
            parent = prim.GetParent()
            while parent and not parent.IsPseudoRoot():
                if parent.HasAPI(UsdPhysics.RigidBodyAPI):
                    errors.append(f"Nested rigid body: {path} inside {parent.GetPath()}")
                    break
                parent = parent.GetParent()
        xform = UsdGeom.Xformable(prim)
        if xform:
            matrix = xform.GetLocalTransformation()
            if not all(finite_vector(row) for row in matrix):
                errors.append(f"Non-finite transform: {path}")
            elif matrix.GetDeterminant() <= 1e-12:
                errors.append(f"Mirrored or degenerate transform: {path}")
        if not prim.IsA(UsdGeom.Mesh):
            continue
        counts["meshes"] += 1
        mesh = UsdGeom.Mesh(prim)
        material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        if not material:
            errors.append(f"Mesh has no resolved material binding: {path}")
        points = mesh.GetPointsAttr().Get() or []
        indices = mesh.GetFaceVertexIndicesAttr().Get() or []
        faces = mesh.GetFaceVertexCountsAttr().Get() or []
        counts["vertices"] += len(points)
        counts["polygons"] += len(faces)
        if not points or not faces:
            errors.append(f"Empty mesh: {path}")
        if any(not finite_vector(p) for p in points):
            errors.append(f"Non-finite vertex: {path}")
        if sum(faces) != len(indices):
            errors.append(f"Face-index count mismatch: {path}")
        if any(i < 0 or i >= len(points) for i in indices):
            errors.append(f"Out-of-range vertex index: {path}")
        if any(n < 3 for n in faces):
            errors.append(f"Degenerate polygon count: {path}")
    return counts


def validate_spatial(stage, manifest, errors):
    """Exact AABB checks for the builder's orthogonal slabs and elevator doors."""
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    result = {"door_center_seams": [], "door_pocket_clearance": [], "shaft_slab_clearance": [],
              "scope": "Orthogonal slab shaft cut-outs, paired door seams and the doors' complete sliding envelopes against shaft sides and cabin jambs. Decorative object intersections and mesh self-intersections are not exhaustively certified."}
    slabs = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh) and "slab" in str(p.GetPath()).lower()]
    for elevator in manifest["elevators"]:
        base = "/World/Elevators/" + elevator["id"]
        obstacles = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)
                     and elevator["id"] + "_" in str(p.GetPath())
                     and any(word in str(p.GetPath()).lower() for word in ("_shaftside", "_sidewall", "_frontjamb", "_frontheader"))]
        pairs = [base + "/Cabin/Doors"] + [base + f"/LandingDoors/Floor_{n}" for n in range(len(elevator["floorHeights"]))]
        for pair in pairs:
            left = stage.GetPrimAtPath(pair + "/Left")
            right = stage.GetPrimAtPath(pair + "/Right")
            if not left or not right:
                continue
            lmin, lmax = world_bounds(cache, left)
            rmin, rmax = world_bounds(cache, right)
            gap = rmin[0] - lmax[0]
            result["door_center_seams"].append({"path": pair, "closed_gap_m": round(gap, 6)})
            if gap < -.015 or gap > .06:
                errors.append(f"Closed door seam outside tolerance: {pair} ({gap:.4f} m)")
            for side, lo, hi in (("Left", lmin, lmax), ("Right", rmin, rmax)):
                swept_lo, swept_hi = list(lo), list(hi)
                travel = float(elevator.get("doorTravel", .72))
                if side == "Left":
                    swept_lo[0] -= travel
                else:
                    swept_hi[0] += travel
                penetrations = []
                for obstacle in obstacles:
                    omin, omax = world_bounds(cache, obstacle)
                    overlap = [min(swept_hi[i], omax[i]) - max(swept_lo[i], omin[i]) for i in range(3)]
                    if all(value > .001 for value in overlap):
                        penetrations.append({"obstacle": str(obstacle.GetPath()), "overlap_m": [round(v, 6) for v in overlap]})
                result["door_pocket_clearance"].append({"path": pair + "/" + side,
                                                        "obstacles_inspected": len(obstacles), "penetrations": penetrations})
                errors.extend(f"Sliding door hits structure: {pair}/{side} -> {p['obstacle']}" for p in penetrations)
        # Cabinet-centre shafts are intentionally pierced through floor slabs.
        # Only check a conservative interior corridor, excluding wall thickness.
        x, y = elevator["position"][:2]
        dims = elevator.get("cabinDimensions", {})
        if isinstance(dims, list):
            dims = dict(zip(("width", "depth", "height"), dims))
        half_width = float(dims.get("width", 2.2)) / 2 - .20
        half_depth = float(dims.get("depth", 2.3)) / 2 - .20
        collisions = []
        inspected = 0
        for slab in slabs:
            lo, hi = world_bounds(cache, slab)
            if hi[2] <= elevator["floorHeights"][0] + .001:
                continue
            if lo[2] >= elevator["floorHeights"][-1] + .05:
                continue
            inspected += 1
            if min(hi[0], x + half_width) - max(lo[0], x - half_width) > .005 and min(hi[1], y + half_depth) - max(lo[1], y - half_depth) > .005:
                collisions.append(str(slab.GetPath()))
        result["shaft_slab_clearance"].append({"elevator": elevator["id"], "slabs_inspected": inspected,
                                                "intersecting_slabs": collisions})
        errors.extend(f"Floor slab blocks shaft {elevator['id']}: {p}" for p in collisions)
    return result


def validate_controls(stage, errors):
    from isaac.usd_adapter import ElevatorScene
    scene = ElevatorScene(stage)
    errors.extend(scene.validate_physics())
    reports = []
    for rig in scene.rigs.values():
        rig.reset()
        target = len(rig.heights) - 1
        display_floors = sorted({floor for _, floor in rig.floor_displays})
        if display_floors != list(range(len(rig.heights))):
            errors.append(f"{rig.elevator_id}: floor display glyphs are missing: {display_floors}")
        destination_button = next((b for b in rig.buttons if b.action == "floor" and b.floor == target), None)
        if destination_button is None:
            raise ValueError(f"{rig.elevator_id}: destination button is missing")
        button_prim = stage.GetPrimAtPath(destination_button.path)
        glyph = next((p for p in Usd.PrimRange(button_prim) if "glyph" in p.GetName().lower()), button_prim)
        dispatched = scene.dispatch_press(str(glyph.GetPath()))
        if not dispatched:
            errors.append(f"{rig.elevator_id}: button/glyph prim-path input dispatch failed")
        requested = [b.floor for b in rig.buttons if b.action == "floor" and b.was_lit]
        if requested != [target]:
            errors.append(f"{rig.elevator_id}: requested cabin lamp isolation failed: {requested}")
        peak_speed = 0.0
        safe = True
        arrived = False
        elapsed = 0.0
        for _ in range(12000):
            state = rig.update(.05)
            elapsed += .05
            peak_speed = max(peak_speed, abs(state["velocity"]))
            if abs(state["velocity"]) > 1e-6 and state["doorOpen"] > 1e-5:
                safe = False
            for floor, pair in enumerate(rig.landing_doors):
                opening = abs(pair[0].op.Get()[0])
                if opening > 1e-5 and (abs(state["position"] - rig.heights[floor]) > 1e-4 or abs(state["velocity"]) > 1e-6):
                    safe = False
            # Requests clear on the completed-opening/dwell transition. Waiting
            # for that state avoids mistaking a 99.9%-open frame for service.
            if state["floor"] == target and state["state"] == "dwell" and state["doorOpen"] == 1.0 and abs(state["velocity"]) < 1e-6:
                arrived = True
                break
        if not safe:
            errors.append(f"{rig.elevator_id}: door travel interlock failed")
        if not arrived:
            errors.append(f"{rig.elevator_id}: failed to arrive at top floor within 600 simulated seconds")
        expected_z = rig.heights[target] - rig.heights[0]
        actual_z = rig.cabin.op.Get()[2]
        if abs(actual_z - expected_z) > 1e-4:
            errors.append(f"{rig.elevator_id}: USD cabin motion mismatch {actual_z} vs {expected_z}")
        cleared = not any(b.was_lit for b in rig.buttons if b.action == "floor")
        if not cleared:
            errors.append(f"{rig.elevator_id}: destination light did not clear on arrival")
        # Verify actual shader values, including the black/off state after use.
        emitted = all(tuple(float(x) for x in emission.Get()) == (0., 0., 0.)
                      for b in rig.buttons if b.action == "floor" for emission in b.emissive_inputs)
        if not emitted:
            errors.append(f"{rig.elevator_id}: arrival shader emissiveColor is not off")
        visible_displays = [floor for display, floor in rig.floor_displays
                            if display.GetVisibilityAttr().Get() != UsdGeom.Tokens.invisible]
        display_valid = set(visible_displays) == {target}
        if not display_valid:
            errors.append(f"{rig.elevator_id}: current floor display did not change to {target}: {visible_displays}")
        reports.append({"elevator": rig.elevator_id, "button_count": len(rig.buttons),
                        "requested_floor": target, "selected_lamp_only": requested == [target],
                        "prim_path_press_dispatched": bool(dispatched), "display_glyph_floors": display_floors,
                        "current_floor_display_updated": display_valid,
                        "arrived": arrived, "arrival_seconds": round(elapsed, 2),
                        "peak_speed_m_s": round(peak_speed, 4), "doors_interlocked": safe,
                        "destination_lamp_cleared": cleared, "cabin_translation_m": float(actual_z)})
        rig.reset()
    return reports


def validate_world(root, name):
    errors, warnings = [], []
    path = root / "scenes" / f"{name}.usda"
    stage = Usd.Stage.Open(str(path))
    if not stage:
        return {"world": name, "passed": False, "errors": [f"Cannot open {path}"]}
    manifest = json.loads((root / "assets" / f"{name}_manifest.json").read_text())
    if not stage.GetDefaultPrim() or str(stage.GetDefaultPrim().GetPath()) != "/World":
        errors.append("Default prim must be /World")
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
        errors.append("Stage must be Z-up")
    if UsdGeom.GetStageMetersPerUnit(stage) != 1:
        errors.append("Stage must use metersPerUnit = 1")
    errors.extend(str(error) for error in stage.GetCompositionErrors())
    layers, assets, unresolved = UsdUtils.ComputeAllDependencies(Sdf.AssetPath(str(path)))
    errors.extend("Unresolved asset: " + str(asset) for asset in unresolved)
    counts = validate_geometry(stage, errors)
    spatial = validate_spatial(stage, manifest, errors)
    controls = validate_controls(stage, errors)
    if not any(item["slabs_inspected"] for item in spatial["shaft_slab_clearance"]):
        warnings.append("No named slab meshes found for shaft cut-out validation")
    return {"world": name, "scene": str(path.relative_to(root)), "passed": not errors,
            "up_axis": str(UsdGeom.GetStageUpAxis(stage)), "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
            "counts": counts, "resolved_layer_count": len(layers), "resolved_asset_count": len(assets),
            "spatial_checks": spatial, "elevator_integration": controls, "errors": errors, "warnings": warnings}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--world", choices=("all", "office", "house"), default="all")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    names = ("office", "house") if args.world == "all" else (args.world,)
    results = [validate_world(args.root, name) for name in names]
    report = {"schema_version": 1, "generated_utc": datetime.now(timezone.utc).isoformat(),
              "usd_version": ".".join(map(str, Usd.GetVersion())),
              "validation_runtime": "OpenUSD CPU; bound Python elevator controller", "isaac_sim_runtime_executed": False,
              "limitations": ["Isaac Sim 5.1.0 GPU/PhysX/RTX execution requires a local supported installation.",
                              "Photorealism requires visual review of rendered output and is not established by topology validation.",
                              "Spatial checks cover orthogonal shaft cut-outs, door seams and sliding pockets, not all possible mesh self-intersections."],
              "passed": all(result["passed"] for result in results), "worlds": results}
    output = args.output or args.root / "docs" / "validation.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"passed": report["passed"], "report": str(output),
                      "errors": {r["world"]: r["errors"] for r in results}}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
