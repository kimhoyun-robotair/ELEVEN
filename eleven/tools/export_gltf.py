#!/usr/bin/env python3
"""Losslessly unpack Blender GLBs into standard glTF with <=8 MiB buffers.

No quantization, mesh simplification, reordering or texture transcoding occurs.
Every bufferView is copied byte-for-byte; only its buffer/byteOffset changes.
This keeps individual repository files below connector transfer size limits.
"""
from pathlib import Path
import hashlib
import json
import struct

ROOT = Path(__file__).resolve().parents[1]
LIMIT = 8 * 1024 * 1024


def unpack(world):
    source = ROOT / "assets" / (world + ".glb")
    raw = source.read_bytes()
    magic, version, total = struct.unpack_from("<III", raw)
    assert magic == 0x46546C67 and version == 2 and total == len(raw)
    offset, document, binary = 12, None, None
    while offset < len(raw):
        size, kind = struct.unpack_from("<II", raw, offset)
        data = raw[offset + 8:offset + 8 + size]
        if kind == 0x4E4F534A:
            document = json.loads(data)
        elif kind == 0x004E4942:
            binary = data
        offset += 8 + size
    assert document is not None and binary is not None
    assert len(document["buffers"]) == 1
    buffers = [bytearray()]
    hashes = []
    for view in document["bufferViews"]:
        assert view["buffer"] == 0
        start, size = view.get("byteOffset", 0), view["byteLength"]
        if size > LIMIT:
            raise ValueError(f"A single bufferView exceeds {LIMIT}: {size}")
        data = binary[start:start + size]
        assert len(data) == size
        padding = (-len(buffers[-1])) % 4
        if len(buffers[-1]) + padding + size > LIMIT:
            buffers.append(bytearray())
            padding = 0
        buffers[-1].extend(b"\0" * padding)
        view["buffer"] = len(buffers) - 1
        view["byteOffset"] = len(buffers[-1])
        buffers[-1].extend(data)
        hashes.append(hashlib.sha256(data).hexdigest())
    document["buffers"] = []
    for index, data in enumerate(buffers):
        filename = f"{world}_{index}.bin"
        (source.parent / filename).write_bytes(data)
        document["buffers"].append({"uri": filename, "byteLength": len(data)})
    output = source.with_suffix(".gltf")
    output.write_text(json.dumps(document, separators=(",", ":")), encoding="utf-8")
    # Verify the delivered bytes, rather than just the in-memory copy.
    for view, expected in zip(document["bufferViews"], hashes):
        buf = (source.parent / document["buffers"][view["buffer"]]["uri"]).read_bytes()
        start = view["byteOffset"]
        assert hashlib.sha256(buf[start:start + view["byteLength"]]).hexdigest() == expected
    path = source.parent / (world + "_manifest.json")
    manifest = json.loads(path.read_text())
    manifest["webAsset"] = world + ".gltf"
    manifest["artifacts"].pop("glb", None)
    manifest["artifacts"]["gltf"] = "assets/" + world + ".gltf"
    manifest["artifacts"]["buffers"] = ["assets/" + b["uri"] for b in document["buffers"]]
    manifest["packaging"] = {"kind": "glTF 2.0 external buffers", "lossless": True,
                             "verifiedBufferViews": len(hashes), "maxBufferBytes": LIMIT}
    path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"{world}: {len(hashes)} bufferViews verified, {len(buffers)} buffers, {output.name}")


if __name__ == "__main__":
    for world in ("office", "house"):
        unpack(world)
