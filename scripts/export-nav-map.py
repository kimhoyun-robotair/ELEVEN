#!/usr/bin/env python3
"""Rasterize the authored Office colliders at the 2D lidar plane into a Nav2 map."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--height", type=float, default=0.192)
args, _ = parser.parse_known_args()
project = Path(__file__).resolve().parents[1]
app = SimulationApp({"headless": True})
try:
    from pxr import Usd, UsdGeom, UsdPhysics
    from PIL import Image, ImageDraw

    stage = Usd.Stage.Open(str(project / "assets/scenes/office.usda"))
    cache = UsdGeom.XformCache()
    resolution, origin, width, height = 0.05, (-15.5, -12.5), 620, 500
    image = Image.new("L", (width, height), 205)
    draw = ImageDraw.Draw(image)
    draw.rectangle((10, 10, width - 11, height - 11), fill=254)
    colliders, segments = [], 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh) or not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
            continue
        if str(prim.GetPath()).startswith("/World/Robot"):
            continue
        mesh = UsdGeom.Mesh(prim)
        points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
        matrix = np.asarray(cache.GetLocalToWorldTransform(prim))
        world = np.column_stack((points, np.ones(len(points)))) @ matrix
        if len(world) == 0 or world[:, 2].min() > args.height or world[:, 2].max() < args.height:
            continue
        indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
        offset, drawn = 0, False
        for count in mesh.GetFaceVertexCountsAttr().Get():
            face = world[indices[offset:offset + count], :3]
            offset += count
            for i in range(1, len(face) - 1):
                triangle = face[[0, i, i + 1]]
                intersections = []
                for a, b in zip(triangle, np.roll(triangle, -1, axis=0)):
                    if (a[2] <= args.height < b[2]) or (b[2] <= args.height < a[2]):
                        ratio = (args.height - a[2]) / (b[2] - a[2])
                        point = a + ratio * (b - a)
                        intersections.append(((point[0] - origin[0]) / resolution,
                                              height - 1 - (point[1] - origin[1]) / resolution))
                if len(intersections) == 2:
                    draw.line(intersections, fill=0, width=2)
                    segments += 1
                    drawn = True
        if drawn:
            colliders.append(str(prim.GetPath()))
    if segments < 100:
        raise RuntimeError(f"Only {segments} collider intersections; refusing an empty map")
    directory = project / "src/aprl_navigation/maps"
    directory.mkdir(parents=True, exist_ok=True)
    output = directory / "office_floor1.pgm"
    image.save(output)
    (directory / "office_floor1.yaml").write_text(
        "image: office_floor1.pgm\nmode: trinary\nresolution: 0.05\n"
        "origin: [-15.5, -12.5, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
    )
    (directory / "office_floor1.source.json").write_text(json.dumps({
        "method": "Authored USD collider triangle intersections, not a SLAM recording",
        "source": "assets/scenes/office.usda", "height_m": args.height,
        "resolution_m": resolution, "origin": origin, "size": [width, height],
        "segments": segments, "colliders": colliders,
        "pgm_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
    }, indent=2) + "\n")
    print(f"NAV_MAP_READY {output}: {len(colliders)} colliders, {segments} segments", flush=True)
finally:
    app.close()
