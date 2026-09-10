"""Structural checks for the delivered Blender-authored glTF assets (stdlib).

Run after generation: python -m unittest discover -s tests -p 'test_glb.py' -v
These tests read assets/{office,house}.gltf, their external .bin buffer shards,
and corresponding manifests. Source .glb files are not required after cloning.
They verify file/buffer integrity, noncollapsed geometry, buffer-backed textures,
Y-up export conversion, and the semantic animation/button contract consumed by
web/dist/app.js. They do not claim visual realism or perform collision tests.
"""

import json
import math
from pathlib import Path
import struct
import unittest


ROOT = Path(__file__).resolve().parents[1]
COMPONENT_BYTES = {5120: 1, 5121: 1, 5122: 2, 5123: 2, 5125: 4, 5126: 4}
TYPE_COMPONENTS = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT2": 4, "MAT3": 9, "MAT4": 16}
IDENTITY = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]


def read_glb(path):
    """Read an optional local source GLB for packaging comparisons, not tests."""
    data = path.read_bytes()
    if len(data) < 20:
        raise ValueError(f"{path}: truncated GLB")
    magic, version, length = struct.unpack_from("<4sII", data)
    if magic != b"glTF" or version != 2 or length != len(data):
        raise ValueError(f"{path}: invalid GLB 2 header/length")
    chunks, offset = [], 12
    while offset < len(data):
        if offset + 8 > len(data):
            raise ValueError(f"{path}: incomplete chunk header")
        size, kind = struct.unpack_from("<II", data, offset)
        offset += 8
        if size % 4 or offset + size > len(data):
            raise ValueError(f"{path}: invalid chunk size")
        chunks.append((kind, data[offset:offset + size]))
        offset += size
    if len(chunks) != 2 or chunks[0][0] != 0x4E4F534A or chunks[1][0] != 0x004E4942:
        raise ValueError(f"{path}: expected JSON plus embedded BIN chunks")
    document = json.loads(chunks[0][1].decode("utf-8").rstrip(" \0\r\n\t"))
    return document, chunks[1][1]


def read_gltf(path):
    """Read the delivered glTF and its local binary buffers without merging offsets."""
    document = json.loads(path.read_text())
    buffers = []
    for buffer in document["buffers"]:
        uri = buffer.get("uri", "")
        if not uri or Path(uri).name != uri or Path(uri).suffix != ".bin":
            raise ValueError(f"{path}: expected a local sibling .bin buffer, got {uri!r}")
        buffers.append((path.parent / uri).read_bytes())
    return document, buffers


def multiply(a, b):
    """Column-major 4x4 matrix product, as specified by glTF."""
    return [sum(a[k * 4 + row] * b[col * 4 + k] for k in range(4)) for col in range(4) for row in range(4)]


def local_matrix(node):
    if "matrix" in node:
        return node["matrix"]
    x, y, z, w = node.get("rotation", [0, 0, 0, 1])
    sx, sy, sz = node.get("scale", [1, 1, 1])
    tx, ty, tz = node.get("translation", [0, 0, 0])
    return [
        (1 - 2 * (y * y + z * z)) * sx, 2 * (x * y + z * w) * sx, 2 * (x * z - y * w) * sx, 0,
        2 * (x * y - z * w) * sy, (1 - 2 * (x * x + z * z)) * sy, 2 * (y * z + x * w) * sy, 0,
        2 * (x * z + y * w) * sz, 2 * (y * z - x * w) * sz, (1 - 2 * (x * x + y * y)) * sz, 0,
        tx, ty, tz, 1,
    ]


def semantic(node, key):
    extras = node.get("extras", {})
    return extras.get("testbed_" + key, extras.get("testbed:" + key))


class GLBStructureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.worlds = {}
        for name in ("office", "house"):
            path = ROOT / "assets" / f"{name}.gltf"
            if not path.is_file():
                raise AssertionError(f"Missing delivered browser asset: {path}")
            doc, buffers = read_gltf(path)
            manifest = json.loads((ROOT / "assets" / f"{name}_manifest.json").read_text())
            cls.worlds[name] = (doc, buffers, manifest)

    def test_gltf_buffers_and_accessors_are_in_bounds(self):
        for name, (doc, buffers, _) in self.worlds.items():
            with self.subTest(world=name):
                self.assertEqual(doc["asset"]["version"], "2.0")
                self.assertGreaterEqual(len(doc["buffers"]), 1)
                self.assertEqual(len(buffers), len(doc["buffers"]))
                for buffer, binary in zip(doc["buffers"], buffers):
                    self.assertEqual(Path(buffer["uri"]).name, buffer["uri"])
                    self.assertGreater(buffer["byteLength"], 0)
                    self.assertEqual(len(binary), buffer["byteLength"])
                    self.assertLessEqual(len(binary), 8 * 1024 * 1024, "binary shard exceeds repository delivery limit")
                for index, view in enumerate(doc["bufferViews"]):
                    self.assertTrue(0 <= view["buffer"] < len(buffers))
                    start = view.get("byteOffset", 0)
                    self.assertGreaterEqual(start, 0)
                    self.assertGreater(view["byteLength"], 0)
                    self.assertLessEqual(start + view["byteLength"], len(buffers[view["buffer"]]), f"{name}: bufferView {index}")
                for index, accessor in enumerate(doc["accessors"]):
                    self.assertGreater(accessor["count"], 0)
                    self.assertIn(accessor["componentType"], COMPONENT_BYTES)
                    self.assertIn(accessor["type"], TYPE_COMPONENTS)
                    self.assertNotIn("sparse", accessor, "unexpected sparse accessor in generated assets")
                    view = doc["bufferViews"][accessor["bufferView"]]
                    element_bytes = COMPONENT_BYTES[accessor["componentType"]] * TYPE_COMPONENTS[accessor["type"]]
                    stride = view.get("byteStride", element_bytes)
                    self.assertGreaterEqual(stride, element_bytes)
                    end = accessor.get("byteOffset", 0) + (accessor["count"] - 1) * stride + element_bytes
                    self.assertLessEqual(end, view["byteLength"], f"{name}: accessor {index}")

    def test_geometry_has_finite_nonzero_dimensions_and_valid_indices(self):
        for name, (doc, buffers, _) in self.worlds.items():
            with self.subTest(world=name):
                self.assertGreater(len(doc["meshes"]), 0)
                checked_indices = set()
                for mesh in doc["meshes"]:
                    mesh_min = [math.inf] * 3
                    mesh_max = [-math.inf] * 3
                    for primitive in mesh["primitives"]:
                        self.assertEqual(primitive.get("mode", 4), 4, "expected triangle meshes")
                        position = doc["accessors"][primitive["attributes"]["POSITION"]]
                        self.assertEqual(position["type"], "VEC3")
                        self.assertEqual(position["componentType"], 5126)
                        self.assertGreaterEqual(position["count"], 3)
                        self.assertTrue(all(math.isfinite(n) for n in position["min"] + position["max"]))
                        # A material primitive can legitimately be a single
                        # planar top face; the complete mesh must have volume.
                        self.assertGreaterEqual(sum(hi - lo > 1e-10 for lo, hi in zip(position["min"], position["max"])), 2, "triangle primitive collapsed to a line or point")
                        for axis in range(3):
                            mesh_min[axis] = min(mesh_min[axis], position["min"][axis])
                            mesh_max[axis] = max(mesh_max[axis], position["max"][axis])
                        if "NORMAL" in primitive["attributes"]:
                            normals = doc["accessors"][primitive["attributes"]["NORMAL"]]
                            self.assertEqual(normals["count"], position["count"])
                        material = primitive.get("material")
                        self.assertIsNotNone(material)
                        self.assertTrue(0 <= material < len(doc["materials"]))
                        indices_id = primitive.get("indices")
                        if indices_id is None:
                            self.assertEqual(position["count"] % 3, 0)
                            continue
                        pair = (indices_id, position["count"])
                        if pair in checked_indices:
                            continue
                        checked_indices.add(pair)
                        accessor = doc["accessors"][indices_id]
                        self.assertEqual(accessor["type"], "SCALAR")
                        self.assertEqual(accessor["count"] % 3, 0)
                        self.assertIn(accessor["componentType"], (5121, 5123, 5125))
                        view = doc["bufferViews"][accessor["bufferView"]]
                        binary = buffers[view["buffer"]]
                        start = view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
                        fmt = "<" + {5121: "B", 5123: "H", 5125: "I"}[accessor["componentType"]]
                        stride = view.get("byteStride", COMPONENT_BYTES[accessor["componentType"]])
                        if stride == COMPONENT_BYTES[accessor["componentType"]]:
                            values = struct.iter_unpack(fmt, memoryview(binary)[start:start + accessor["count"] * stride])
                            maximum = max(value[0] for value in values)
                        else:
                            maximum = max(struct.unpack_from(fmt, binary, start + i * stride)[0] for i in range(accessor["count"]))
                        self.assertLess(maximum, position["count"], f"{name}: invalid triangle index")
                    for minimum, maximum in zip(mesh_min, mesh_max):
                        self.assertGreater(maximum - minimum, 1e-10, f"{name}: collapsed mesh {mesh.get('name')}")

    def test_scene_tree_transforms_and_y_up_export(self):
        for name, (doc, _, manifest) in self.worlds.items():
            with self.subTest(world=name):
                nodes = doc["nodes"]
                world_matrices, visited = {}, set()
                stack = [(index, IDENTITY) for index in doc["scenes"][doc.get("scene", 0)]["nodes"]]
                while stack:
                    index, parent_matrix = stack.pop()
                    self.assertNotIn(index, visited, "scene nodes must have one parent and no cycles")
                    self.assertTrue(0 <= index < len(nodes))
                    visited.add(index)
                    node = nodes[index]
                    matrix = local_matrix(node)
                    self.assertEqual(len(matrix), 16)
                    self.assertTrue(all(math.isfinite(value) for value in matrix))
                    determinant = (matrix[0] * (matrix[5] * matrix[10] - matrix[9] * matrix[6])
                                   - matrix[4] * (matrix[1] * matrix[10] - matrix[9] * matrix[2])
                                   + matrix[8] * (matrix[1] * matrix[6] - matrix[5] * matrix[2]))
                    self.assertGreater(abs(determinant), 1e-12, f"{name}: collapsed node transform")
                    world_matrices[index] = multiply(parent_matrix, matrix)
                    stack.extend((child, world_matrices[index]) for child in node.get("children", []))
                self.assertEqual(len(visited), len(nodes), "orphan nodes must not silently disappear")
                paths = {semantic(node, "path"): index for index, node in enumerate(nodes) if semantic(node, "path")}
                self.assertEqual(manifest["upAxis"], "Z", "manifest describes Blender/Isaac coordinates")
                self.assertEqual(manifest["units"], "meters")
                for lift in manifest["elevators"]:
                    prefix = "/World/Elevators/" + lift["id"]
                    x, y, z = lift["position"]
                    # Blender export_yup maps source (X,Y,Z) to glTF (X,Z,-Y).
                    actual = world_matrices[paths[prefix]][12:15]
                    for value, expected in zip(actual, (x, z, -y)):
                        self.assertAlmostEqual(value, expected, places=5)
                    cabin = world_matrices[paths[prefix + "/Cabin"]][12:15]
                    for value, expected in zip(cabin, (x, z, -y)):
                        self.assertAlmostEqual(value, expected, places=5)
                    for floor, height in enumerate(manifest["floorHeights"]):
                        landing = world_matrices[paths[f"{prefix}/LandingDoors/Floor_{floor}"]][12:15]
                        for value, expected in zip(landing, (x, z + height, -y)):
                            self.assertAlmostEqual(value, expected, places=5)

    def test_animation_buttons_displays_and_floor_groups_match_manifest(self):
        for name, (doc, _, manifest) in self.worlds.items():
            with self.subTest(world=name):
                nodes = doc["nodes"]
                paths, names, parents = {}, {}, {}
                for index, node in enumerate(nodes):
                    names[node["name"]] = index
                    path = semantic(node, "path")
                    if path:
                        self.assertNotIn(path, paths, "animation semantic paths must be unique")
                        paths[path] = index
                    for child in node.get("children", []):
                        parents[child] = index

                def descends(child, ancestor):
                    while child in parents:
                        child = parents[child]
                        if child == ancestor:
                            return True
                    return False

                heights = manifest["floorHeights"]
                elevator_ids = {lift["id"] for lift in manifest["elevators"]}
                for node in nodes:
                    elevator_id = semantic(node, "elevatorId")
                    if elevator_id is not None:
                        self.assertIn(elevator_id, elevator_ids, "foreign elevator metadata survived scene cleanup")
                self.assertTrue(all(b > a for a, b in zip(heights, heights[1:])))
                self.assertEqual({lift["type"] for lift in manifest["elevators"]}, {"opaque", "glass"})
                self.assertIn(manifest["roofNode"], names)
                for name_key in ("floorGroups", "furnitureFloorGroups"):
                    self.assertEqual(len(manifest[name_key]), len(heights))
                    for floor, node_name in enumerate(manifest[name_key]):
                        self.assertEqual(semantic(nodes[names[node_name]], "floorGroup"), floor)
                lamp_materials = set()
                for lift in manifest["elevators"]:
                    prefix = "/World/Elevators/" + lift["id"]
                    self.assertEqual(semantic(nodes[paths[prefix]], "type"), lift["type"])
                    cabin = paths[prefix + "/Cabin"]
                    for side in ("Left", "Right"):
                        door = paths[prefix + "/Cabin/Doors/" + side]
                        self.assertTrue(descends(door, cabin), "cabin doors must follow cabin travel")
                        self.assertTrue(nodes[door].get("children"), "door group must animate actual geometry")
                    for floor in range(len(heights)):
                        for side in ("Left", "Right"):
                            door = paths[f"{prefix}/LandingDoors/Floor_{floor}/{side}"]
                            self.assertFalse(descends(door, cabin), "landing doors must remain at their storey")
                            self.assertTrue(nodes[door].get("children"))
                    signatures = set()
                    for button in lift["buttons"]:
                        lamp_index = names[button["node"]]
                        lamp = nodes[lamp_index]
                        group = paths[button["path"]]
                        self.assertTrue(descends(lamp_index, group))
                        self.assertEqual(semantic(lamp, "elevatorId"), lift["id"])
                        self.assertEqual(semantic(lamp, "button"), button["type"])
                        self.assertEqual(semantic(lamp, "floor"), button["floor"])
                        self.assertEqual(semantic(lamp, "direction"), button["direction"])
                        self.assertIn("mesh", lamp, "interactive lamp must be a rendered mesh")
                        materials = {p["material"] for p in doc["meshes"][lamp["mesh"]]["primitives"]}
                        self.assertEqual(len(materials), 1)
                        material = next(iter(materials))
                        self.assertNotIn(material, lamp_materials, "each lamp needs its own material for independent USD emission")
                        lamp_materials.add(material)
                        emission = doc["materials"][material].get("emissiveFactor", [0, 0, 0])
                        self.assertEqual(emission, [0, 0, 0], "unpressed request lamps must start dark")
                        signatures.add((button["type"], button["floor"], button["direction"]))
                    expected = {("floor", floor, None) for floor in range(len(heights))}
                    expected.update((kind, None, None) for kind in ("open", "close", "alarm"))
                    expected.update(("hall", floor, "up") for floor in range(len(heights) - 1))
                    expected.update(("hall", floor, "down") for floor in range(1, len(heights)))
                    self.assertEqual(signatures, expected)
                    display_nodes = [(index, node) for index, node in enumerate(nodes) if semantic(node, "elevatorId") == lift["id"] and semantic(node, "displayFloor") is not None]
                    self.assertEqual(len(display_nodes), len(heights) * (len(heights) + 1), "each cabin and landing needs one glyph for each display floor")
                    display_groups = {}
                    for index, node in display_nodes:
                        self.assertIn(index, parents, "display glyph cannot be an orphan scene root")
                        display_groups.setdefault(parents[index], []).append(node)
                    self.assertEqual(len(display_groups), len(heights) + 1)
                    cabin_displays = 0
                    hall_display_floors = set()
                    for parent, group_nodes in display_groups.items():
                        self.assertEqual(len(group_nodes), len(heights))
                        self.assertEqual({semantic(node, "displayFloor") for node in group_nodes}, set(range(len(heights))), "every display needs all digits for runtime switching")
                        if parent == cabin or descends(parent, cabin):
                            cabin_displays += 1
                        else:
                            possible_floors = [f for f in range(len(heights)) if nodes[parent]["name"] == f"{lift['id']}_Shaft_Floor_{f}"]
                            self.assertEqual(len(possible_floors), 1, "hall displays must belong to their elevator landing")
                            landing_floor = possible_floors[0]
                            self.assertTrue(descends(parent, names[manifest["floorGroups"][landing_floor]]))
                            hall_display_floors.add(landing_floor)
                    self.assertEqual(cabin_displays, 1)
                    self.assertEqual(hall_display_floors, set(range(len(heights))))

    def test_textures_are_buffer_backed_and_pbr_materials_include_glass(self):
        for name, (doc, buffers, _) in self.worlds.items():
            with self.subTest(world=name):
                self.assertGreater(len(doc.get("images", [])), 0)
                for image in doc["images"]:
                    self.assertNotIn("uri", image, "demo textures must be embedded in delivered binary buffers")
                    self.assertIn(image["mimeType"], ("image/png", "image/jpeg"))
                    view = doc["bufferViews"][image["bufferView"]]
                    binary = buffers[view["buffer"]]
                    offset = view.get("byteOffset", 0)
                    data = binary[offset:offset + view["byteLength"]]
                    if image["mimeType"] == "image/png":
                        self.assertEqual(data[:8], b"\x89PNG\r\n\x1a\n")
                        width, height = struct.unpack_from(">II", data, 16)
                        self.assertGreater(width, 0)
                        self.assertGreater(height, 0)
                    else:
                        self.assertEqual(data[:2], b"\xff\xd8")
                        self.assertEqual(data[-2:], b"\xff\xd9")
                for texture in doc["textures"]:
                    self.assertTrue(0 <= texture["source"] < len(doc["images"]))
                self.assertTrue(any("baseColorTexture" in m.get("pbrMetallicRoughness", {}) for m in doc["materials"]))
                self.assertTrue(any("normalTexture" in m for m in doc["materials"]))
                self.assertTrue(any("metallicRoughnessTexture" in m.get("pbrMetallicRoughness", {}) for m in doc["materials"]))
                self.assertTrue(any(m.get("extensions", {}).get("KHR_materials_transmission", {}).get("transmissionFactor", 0) > 0.5 for m in doc["materials"]), "glass elevators need transmissive material")


if __name__ == "__main__":
    unittest.main()
