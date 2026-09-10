#!/usr/bin/env python3
"""Copy delivered glTF files, buffers and controller into the static demo."""
from pathlib import Path
import shutil
import json

ROOT = Path(__file__).resolve().parents[1]
dest = ROOT / "web/dist/assets"
dest.mkdir(parents=True, exist_ok=True)
for world in ("office", "house"):
    for suffix in (".gltf", "_manifest.json"):
        shutil.copyfile(ROOT / "assets" / (world + suffix), dest / (world + suffix))
    gltf = json.loads((ROOT / "assets" / (world + ".gltf")).read_text())
    for buffer in gltf["buffers"]:
        shutil.copyfile(ROOT / "assets" / buffer["uri"], dest / buffer["uri"])
    old_glb = dest / (world + ".glb")
    if old_glb.exists():
        old_glb.unlink()
shutil.copyfile(ROOT / "simulation/elevator.mjs", ROOT / "web/dist/elevator.mjs")
print("Web assets synchronized with generated worlds and controller.")
