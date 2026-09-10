#!/usr/bin/env python3
"""Turn Blender's metre/Z-up USD exports into interactive Isaac Sim worlds.

Run with a Python interpreter providing ``pxr`` (Isaac Sim's python.sh or
``pip install usd-core``). This preparation step does not start Isaac Sim.
The original .blend and GLB assets remain the authoring and web counterparts.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

ROOT = Path(__file__).resolve().parents[1]


def custom_value(prim, key, default=None):
    """Blender versions differ in their custom-property namespace."""
    for name in (key, "userProperties:" + key, "userProperties:blender:" + key):
        attr = prim.GetAttribute(name)
        if attr and attr.Get() is not None:
            return attr.Get()
    for attr in prim.GetAttributes():
        if attr.GetName().split(":")[-1] == key:
            value = attr.Get()
            if value is not None:
                return value
    return default


def attr(prim, name, type_name, value):
    return prim.CreateAttribute(name, type_name, custom=True).Set(value)


def find_named(stage, name):
    matches = [p for p in stage.Traverse() if p.GetName() == name]
    if len(matches) == 1:
        return matches[0]
    return None


def move_prim(stage, source, destination):
    source, destination = str(source), str(destination)
    if source == destination:
        return
    if stage.GetPrimAtPath(destination):
        raise ValueError(f"Cannot rename {source}: {destination} already exists")
    parent = Sdf.Path(destination).GetParentPath()
    if not stage.GetPrimAtPath(parent):
        UsdGeom.Xform.Define(stage, parent)
    editor = Usd.NamespaceEditor(stage)
    editor.MovePrimAtPath(source, destination)
    if not editor.CanApplyEdits():
        raise RuntimeError(f"USD namespace edit failed: {source} -> {destination}")
    if not editor.ApplyEdits():
        raise RuntimeError(f"USD namespace edit was not applied: {source} -> {destination}")


def canonicalize_geometry(geometry, manifest):
    """Persist canonical semantic names, including all rewritten relationships."""
    stage = Usd.Stage.Open(str(geometry))
    if not stage:
        raise ValueError(f"Cannot open {geometry}")
    # Blender 4.x commonly introduces /root above the authored World object,
    # placing materials and cameras alongside it. Keep the complete deliverable
    # below one default prim so it can also be referenced into another scene.
    if not stage.GetPrimAtPath("/World"):
        authored_world = find_named(stage, "World")
        if not authored_world:
            raise ValueError("Blender USD export has no uniquely named World object")
        wrapper = authored_world.GetParent()
        wrapper_path = str(wrapper.GetPath())
        if not wrapper.IsPseudoRoot():
            wrapper_xform = UsdGeom.Xformable(wrapper)
            if wrapper_xform and wrapper_xform.GetLocalTransformation() != Gf.Matrix4d(1):
                raise ValueError("Unexpected transformed export wrapper: " + wrapper_path)
        move_prim(stage, authored_world.GetPath(), "/World")
        if wrapper_path != "/":
            wrapper = stage.GetPrimAtPath(wrapper_path)
            for child_path in [str(p.GetPath()) for p in wrapper.GetChildren()]:
                child = stage.GetPrimAtPath(child_path)
                name = child.GetName()
                if "material" in name.lower():
                    target = "/World/Looks"
                elif child.IsA(UsdLux.DomeLight):
                    target = "/World/Lighting/BlenderEnvironment"
                else:
                    target = "/World/Cameras/" + name
                move_prim(stage, child_path, target)
            if not stage.GetPrimAtPath(wrapper_path).GetChildren():
                stage.RemovePrim(wrapper_path)
    # Semantic paths are authored on Blender grouping objects and are preferable
    # to guessing how a particular exporter sanitized globally unique names.
    semantic_paths = {}
    for prim in stage.Traverse():
        target = custom_value(prim, "testbed_path")
        if target:
            semantic_paths.setdefault(str(target), str(prim.GetPath()))
    desired = sorted(semantic_paths, key=lambda p: (p.count("/"), p))
    print(f"Canonicalizing {geometry.name}: {len(desired)} semantic groups", flush=True)
    for target in desired:
        source = semantic_paths[target]
        if source == target or stage.GetPrimAtPath(target):
            continue
        move_prim(stage, source, target)
        # Keep tracked descendant paths current after each ancestor rename;
        # avoid repeatedly walking every furnished mesh for every control.
        for key, path in semantic_paths.items():
            if path == source or path.startswith(source + "/"):
                semantic_paths[key] = target + path[len(source):]
    # Fallback for exporters that do not preserve custom object properties.
    for elevator in manifest["elevators"]:
        eid = elevator["id"]
        base = "/World/Elevators/" + eid
        mapping = [(elevator.get("cabinNode", eid + "_Cabin"), base + "/Cabin"),
                   (eid + "_Cabin_Doors", base + "/Cabin/Doors"),
                   (eid + "_LandingDoors", base + "/LandingDoors")]
        doors = elevator.get("cabinDoorNodes", {})
        for side in ("left", "right"):
            mapping.append((doors.get(side, eid + "_Cabin_Door_" + side.title()),
                            base + "/Cabin/Doors/" + side.title()))
        for floor in range(len(elevator["floorHeights"])):
            mapping.append((eid + f"_Landing_Floor_{floor}", base + f"/LandingDoors/Floor_{floor}"))
            entry = next((d for d in elevator.get("landingDoorNodes", []) if d["floor"] == floor), {})
            for side in ("left", "right"):
                mapping.append((entry.get(side, eid + f"_Landing_F{floor}_" + side.title()),
                                base + f"/LandingDoors/Floor_{floor}/" + side.title()))
        for name, target in mapping:
            if stage.GetPrimAtPath(target):
                continue
            source = find_named(stage, name)
            if source:
                move_prim(stage, source.GetPath(), target)
        if not stage.GetPrimAtPath(base + "/Cabin"):
            raise ValueError(f"Missing elevator cabin in Blender geometry: {base}")
    world = stage.GetPrimAtPath("/World")
    if not world:
        raise ValueError("Blender USD export must contain /World")
    stage.SetDefaultPrim(world)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.GetRootLayer().Save()


def collision_mesh(prim):
    UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
    if prim.IsA(UsdGeom.Mesh):
        # Independent triangle meshes preserve door openings and shaft voids.
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("none")


def kinematic(prim):
    api = UsdPhysics.RigidBodyAPI.Apply(prim)
    api.CreateRigidBodyEnabledAttr(True)
    api.CreateKinematicEnabledAttr(True)


def collision_box(stage, path, center, size):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*center))
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreatePurposeAttr(UsdGeom.Tokens.guide)
    cube.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    collision_mesh(cube.GetPrim())
    return cube


def lamp_material(stage, path):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.13, 0.16, 0.18))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.42)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.24)
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0))
    shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def static_collision_eligible(prim):
    if not prim.IsA(UsdGeom.Mesh):
        return False
    path = str(prim.GetPath())
    if "/Cabin/" in path or "/LandingDoors/" in path:
        return False
    explicit = custom_value(prim, "testbed_collision")
    if explicit is not None:
        return bool(explicit)
    name = path.lower()
    # Furniture remains solid to robots; leaf cards, text, light diffusers and
    # small control-panel decoration do not create gratuitous collider counts.
    return not any(word in name for word in (
        "text", "label", "lamp", "button", "indicator", "led_", "light",
        "plant", "leaf", "foliage", "screen", "artwork", "picture", "book",
        "handle", "rail", "trim", "mullion", "curtain", "blind", "panel_digit"))


def configure_elevator(stage, elevator):
    eid = elevator["id"]
    base = "/World/Elevators/" + eid
    root = stage.GetPrimAtPath(base)
    if not root:
        raise ValueError("Missing " + base)
    heights = [float(h) for h in elevator["floorHeights"]]
    attr(root, "testbed:floorHeights", Sdf.ValueTypeNames.DoubleArray, heights)
    attr(root, "testbed:doorTravel", Sdf.ValueTypeNames.Double, float(elevator.get("doorTravel", .72)))
    attr(root, "testbed:label", Sdf.ValueTypeNames.String, elevator.get("label", eid))
    attr(root, "testbed:type", Sdf.ValueTypeNames.Token, elevator["type"])
    attr(root, "testbed:currentFloor", Sdf.ValueTypeNames.Int, 0)
    attr(root, "testbed:state", Sdf.ValueTypeNames.String, "IDLE")
    cabin = stage.GetPrimAtPath(base + "/Cabin")
    if not cabin:
        raise ValueError("Missing " + base + "/Cabin")
    # Structural collision proxies move together as one body, while doors have
    # their own bodies. No rigid-body parent contains another rigid body.
    dimensions = elevator.get("cabinDimensions", {})
    if isinstance(dimensions, list):
        dimensions = dict(zip(("width", "depth", "height"), dimensions))
    width = float(dimensions.get("width", 2.4))
    depth = float(dimensions.get("depth", 2.4))
    height = float(dimensions.get("height", 2.7))
    collision = UsdGeom.Xform.Define(stage, base + "/Cabin/Collision")
    kinematic(collision.GetPrim())
    # These authored dimensions match the Blender cabinet exactly; the floor
    # top is at the nominal landing elevation. A manifest may override all boxes.
    boxes = elevator.get("cabinCollisionBoxes", [
        {"name": "Floor", "center": (0, 0, -.07), "size": (width, depth, .14)},
        {"name": "Back", "center": (0, 1.15, height / 2), "size": (2.28, .09, height)},
        {"name": "LeftWall", "center": (-1.11, .03, height / 2), "size": (.085, 2.29, height)},
        {"name": "RightWall", "center": (1.11, .03, height / 2), "size": (.085, 2.29, height)},
        {"name": "Ceiling", "center": (0, 0, 2.685), "size": (width, depth, .09)},
        {"name": "FrontJambLeft", "center": (-.966, -1.15, 1.16), "size": (.46, .11, 2.32)},
        {"name": "FrontJambRight", "center": (.966, -1.15, 1.16), "size": (.46, .11, 2.32)},
        {"name": "FrontHeader", "center": (0, -1.15, 2.50), "size": (width, .11, .36)},
    ])
    for box in boxes:
        collision_box(stage, base + "/Cabin/Collision/" + box["name"], box["center"], box["size"])
    door_paths = [base + "/Cabin/Doors/" + side for side in ("Left", "Right")]
    door_paths += [base + f"/LandingDoors/Floor_{floor}/" + side
                   for floor in range(len(heights)) for side in ("Left", "Right")]
    for path in door_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim:
            raise ValueError("Missing " + path)
        kinematic(prim)
        meshes = [p for p in Usd.PrimRange(prim) if p.IsA(UsdGeom.Mesh)]
        if not meshes:
            raise ValueError("Door has no mesh: " + path)
        for mesh in meshes:
            collision_mesh(mesh)
    for button in elevator["buttons"]:
        lamp = find_named(stage, button["node"])
        prim = stage.GetPrimAtPath(button["path"]) if button.get("path") else None
        if not prim and button.get("groupNode"):
            prim = find_named(stage, button["groupNode"])
        if not prim:
            prim = lamp
        if not prim or not lamp:
            raise ValueError(f"Missing button {eid}: {button['node']}")
        action = button.get("type", button.get("button", "floor"))
        attr(prim, "testbed:elevatorId", Sdf.ValueTypeNames.String, eid)
        attr(prim, "testbed:button", Sdf.ValueTypeNames.Token, action)
        attr(prim, "testbed:floor", Sdf.ValueTypeNames.Int, int(button.get("floor") or 0))
        attr(prim, "testbed:direction", Sdf.ValueTypeNames.Token, button.get("direction") or "up")
        safe_name = Tf.MakeValidIdentifier(button["node"])
        material = lamp_material(stage, "/World/TestbedMaterials/" + safe_name)
        UsdShade.MaterialBindingAPI.Apply(lamp).Bind(material)
        prim.CreateRelationship("testbed:lightMaterial", custom=True).SetTargets([material.GetPath()])
        # Exported mesh children often carry stronger local material bindings.
        for descendant in Usd.PrimRange(lamp):
            if descendant.IsA(UsdGeom.Mesh):
                UsdShade.MaterialBindingAPI.Apply(descendant).Bind(material)


def configure_interior_lighting(stage):
    for prim in stage.Traverse():
        if not prim.IsA(UsdLux.DiskLight):
            continue
        if prim.GetName().startswith(("House_ceiling_light", "Office_ceiling_light")):
            UsdLux.LightAPI(prim).CreateExposureAttr(11.0)
        elif "_Downlight" in prim.GetName():
            UsdLux.LightAPI(prim).CreateExposureAttr(11.0)


def prepare_world(root, world):
    manifest_path = root / "assets" / f"{world}_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    geometry = root / "scenes" / f"{world}_geometry.usdc"
    canonicalize_geometry(geometry, manifest)
    scene_path = root / "scenes" / f"{world}.usda"
    if scene_path.exists():
        scene_path.unlink()
    stage = Usd.Stage.CreateNew(str(scene_path))
    stage.SetMetadata("documentation", "Blender-authored multi-floor testbed; run isaac/run.py for active elevators. Prepared with OpenUSD; Isaac Sim 5.1 runtime validation is a separate step.")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    # Preserve exporter-owned root materials/cameras as well as /World. Some
    # Blender releases place their material scope beside the World object.
    stage.GetRootLayer().subLayerPaths = [geometry.name]
    world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world_prim)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(240)
    stage.SetTimeCodesPerSecond(60)
    physics = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    physics.CreateGravityMagnitudeAttr(9.81)
    attr(physics.GetPrim(), "testbed:targetIsaacSim", Sdf.ValueTypeNames.String, "5.1.0")
    for prim in stage.Traverse():
        if static_collision_eligible(prim):
            collision_mesh(prim)
        display_floor = custom_value(prim, "testbed_displayFloor")
        if display_floor is not None:
            attr(prim, "testbed:displayFloor", Sdf.ValueTypeNames.Int, int(display_floor))
            display_elevator = custom_value(prim, "testbed_elevatorId")
            if display_elevator:
                attr(prim, "testbed:elevatorId", Sdf.ValueTypeNames.String, str(display_elevator))
            UsdGeom.Imageable(prim).CreateVisibilityAttr().Set(
                UsdGeom.Tokens.inherited if int(display_floor) == 0 else UsdGeom.Tokens.invisible)
    for elevator in manifest["elevators"]:
        configure_elevator(stage, elevator)
    configure_interior_lighting(stage)
    # A neutral sky complements authored area lights and provides environment
    # illumination without an externally hosted HDRI dependency.
    dome = UsdLux.DomeLight.Define(stage, "/World/Lighting/Environment")
    dome.CreateIntensityAttr(420)
    dome.CreateColorAttr(Gf.Vec3f(.79, .87, 1.0))
    sun = UsdLux.DistantLight.Define(stage, "/World/Lighting/Sun")
    sun.CreateIntensityAttr(2100)
    sun.CreateAngleAttr(.7)
    sun.CreateColorAttr(Gf.Vec3f(1.0, .91, .79))
    sun.AddRotateXYZOp().Set(Gf.Vec3f(30, -28, -38))
    camera = UsdGeom.Camera.Define(stage, "/World/Cameras/Overview")
    target = Gf.Vec3d(0, 1, 4 if world == "office" else 3)
    position = Gf.Vec3d(29, -36, 25) if world == "office" else Gf.Vec3d(23, -29, 19)
    view = Gf.Matrix4d().SetLookAt(position, target, Gf.Vec3d(0, 0, 1))
    camera.AddTransformOp().Set(view.GetInverse())
    camera.CreateFocalLengthAttr(28)
    camera.CreateClippingRangeAttr(Gf.Vec2f(.05, 5000))
    stage.GetRootLayer().Save()
    print(f"Prepared {scene_path.relative_to(root)}")
    return scene_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", choices=("all", "office", "house"), default="all")
    parser.add_argument("--root", type=Path, default=ROOT)
    args = parser.parse_args()
    for world in (("office", "house") if args.world == "all" else (args.world,)):
        prepare_world(args.root, world)


if __name__ == "__main__":
    main()
