#!/usr/bin/env python3
"""Losslessly package large Blender USD exports into furniture-floor sublayers.

The geometry entry filename, every prim path, all authored values and traversal
order remain unchanged. A complete composed-data SHA-256 comparison is made
before the original entry layer is atomically replaced. This is packaging only:
it neither decimates nor re-exports geometry through Blender.

Run after prepare_isaac.py, then run validate_assets.py. By default, geometry
entry files below 11 MiB are retained as they are.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path

from pxr import Sdf, Usd

ROOT = Path(__file__).resolve().parents[1]


def _hash_value(digest, value):
    """Retain exact typed mesh buffers instead of comparing printed floats."""
    digest.update((type(value).__name__ + "\0").encode())
    if isinstance(value, dict):
        for key in sorted(value):
            _hash_value(digest, key)
            _hash_value(digest, value[key])
    elif isinstance(value, (list, tuple)):
        for item in value:
            _hash_value(digest, item)
    else:
        try:
            buffer = memoryview(value)
        except TypeError:
            digest.update(repr(value).encode("utf-8"))
        else:
            digest.update(repr((buffer.format, buffer.shape)).encode())
            digest.update(buffer.tobytes())
    digest.update(b"\0")


def composed_digest(stage):
    """Hash every flattened spec/field, then the actual composed child order.

    Prim-order *opinions* may be added to preserve existing composed ordering
    across sublayers. Their resulting order is checked, while USDA formatting
    and layer provenance are intentionally excluded. All other fields remain:
    types, metadata, exact mesh buffers, transforms, connections, relationships,
    time samples, material bindings and anchored texture asset paths.
    """
    flattened = stage.Flatten(False)
    paths = []
    flattened.Traverse(Sdf.Path.absoluteRootPath, paths.append)
    digest = hashlib.sha256()
    for path in sorted(paths, key=str):
        spec = flattened.GetObjectAtPath(path)
        _hash_value(digest, str(path))
        _hash_value(digest, type(spec).__name__)
        # Sdf traversal also visits relationship-target pseudo paths, which
        # have no independent Spec object; the parent relationship fields and
        # the target path above are still included in the digest.
        if spec is None:
            continue
        for key in sorted(spec.ListInfoKeys()):
            if key == "primOrder":
                continue
            _hash_value(digest, key)
            _hash_value(digest, spec.GetInfo(key))
    for prim in sorted(stage.TraverseAll(), key=lambda p: str(p.GetPath())):
        _hash_value(digest, str(prim.GetPath()))
        _hash_value(digest, [child.GetName() for child in prim.GetAllChildren()])
    return digest.hexdigest()


def split_geometry(path, max_bytes):
    path = path.resolve()
    if path.stat().st_size < max_bytes:
        return {"entry": path.name, "split": False, "bytes": path.stat().st_size}
    source_stage = Usd.Stage.Open(str(path))
    source = source_stage.GetRootLayer()
    furniture = source.GetPrimAtPath("/World/Furniture")
    floors = [prim for prim in furniture.nameChildren.values()
              if prim.name.startswith("Furniture_Floor_")] if furniture else []
    if not floors:
        raise ValueError(f"{path.name} exceeds the size threshold but has no authored furniture floors")
    before = composed_digest(source_stage)
    prefix = path.stem.removesuffix("_geometry")
    pending = path.with_name(path.stem + "_packaging_pending.usdc")
    if pending.exists():
        raise FileExistsError(f"Refusing to overwrite an existing temporary layer: {pending}")
    working = Sdf.Layer.CreateAnonymous("packaged_geometry")
    working.TransferContent(source)
    parts = []
    try:
        for floor in floors:
            index = int(floor.name.removeprefix("Furniture_Floor_"))
            part_path = path.with_name(f"{prefix}_furniture_f{index}.usdc")
            if part_path.exists():
                raise FileExistsError(f"Refusing to overwrite an existing package part: {part_path}")
            part = Sdf.Layer.CreateNew(str(part_path))
            parts.append(part_path)
            Sdf.CreatePrimInLayer(part, floor.path.GetParentPath())
            if not Sdf.CopySpec(source, floor.path, part, floor.path):
                raise RuntimeError(f"Could not copy exact authored specs for {floor.path}")
            part.Save()
            if part_path.stat().st_size >= max_bytes:
                raise ValueError(f"Furniture part is still too large: {part_path.name}")
            edit = Sdf.BatchNamespaceEdit()
            edit.Add(floor.path, Sdf.Path.emptyPath)
            if not working.Apply(edit):
                raise RuntimeError(f"Could not remove copied source specs for {floor.path}")
        # USD merges child order from weaker to stronger layers. Reverse the
        # sublayer strength list to retain the original furniture-floor order.
        working.subLayerPaths = [p.name for p in reversed(parts)] + list(source.subLayerPaths)
        # Partial weaker ancestors otherwise put Furniture before Building.
        # Preserve the original *composed* order explicitly at both ancestors.
        for ancestor in ("/World", "/World/Furniture"):
            working.GetPrimAtPath(ancestor).nameChildrenOrder = [
                child.GetName() for child in source_stage.GetPrimAtPath(ancestor).GetAllChildren()]
        if not working.Export(str(pending)):
            raise RuntimeError(f"Could not write {pending.name}")
        if pending.stat().st_size >= max_bytes:
            raise ValueError(f"Entry layer remains above the threshold after splitting: {pending.stat().st_size}")
        packaged_stage = Usd.Stage.Open(str(pending))
        if packaged_stage.GetCompositionErrors():
            raise RuntimeError(str(packaged_stage.GetCompositionErrors()))
        after = composed_digest(packaged_stage)
        if before != after:
            raise RuntimeError("Lossless packaging check failed: composed fields or child order changed")
        old_bytes = path.stat().st_size
        os.replace(pending, path)
        pending.unlink(missing_ok=True)
        return {"entry": path.name, "split": True, "original_bytes": old_bytes,
                "bytes": path.stat().st_size, "composed_data_sha256": after,
                "lossless_composition_comparison": True,
                "parts": [{"file": p.name, "bytes": p.stat().st_size} for p in parts]}
    except Exception:
        pending.unlink(missing_ok=True)
        for part in parts:
            part.unlink(missing_ok=True)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--world", choices=("all", "office", "house"), default="all")
    parser.add_argument("--threshold-mib", type=float, default=11.0)
    args = parser.parse_args()
    if args.threshold_mib <= 0:
        parser.error("--threshold-mib must be positive")
    worlds = ("office", "house") if args.world == "all" else (args.world,)
    maximum = int(args.threshold_mib * 1024 * 1024)
    results = [split_geometry(args.root / "scenes" / f"{name}_geometry.usdc", maximum)
               for name in worlds]
    print(json.dumps({"maximum_file_bytes": maximum, "worlds": results}, indent=2))


if __name__ == "__main__":
    main()
