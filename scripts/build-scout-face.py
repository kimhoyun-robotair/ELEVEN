#!/usr/bin/env python3
"""Build the optional Scout face with Blender 4 and compose its ROS/USD assets."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys

PROJECT = Path(__file__).resolve().parents[1]
COMPONENTS = PROJECT / 'assets/robot/components'
PACKAGE = PROJECT / 'src/scout_twin_description'
REFERENCE = PACKAGE / 'materials/creeper_reference.png'
NAME = 'scout_creeper_face'


def model_in_blender():
    import bpy
    from statistics import median
    from mathutils import Vector

    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    scene.unit_settings.system = 'METRIC'
    source = bpy.data.images.load(str(REFERENCE))
    source.pack()
    width, height = source.size
    pixels = list(source.pixels)

    def sample(row, col):
        # Rectify the visible front's 8x8 cells in the supplied isometric PNG.
        u, v = (col + .5) / 8, (row + .5) / 8
        x, y = round(83 + 67 * u), round(150 + 33 * u + 81 * v)
        if (width, height) != (300, 300):
            raise ValueError('Expected the supplied 300x300 Creeper reference')
        return tuple(median(pixels[((height - 1 - y - dy) * width + x + dx) * 4 + c]
                            for dy in (-1, 0, 1) for dx in (-1, 0, 1)) for c in range(3))

    def material(name, rgb, roughness):
        mat = bpy.data.materials.new(name)
        linear = tuple(c / 12.92 if c <= .04045 else ((c + .055) / 1.055) ** 2.4 for c in rgb)
        mat.diffuse_color = (*linear, 1)
        mat['source_srgb'] = list(rgb)
        mat.use_nodes = True
        shader = mat.node_tree.nodes.get('Principled BSDF')
        shader.inputs['Base Color'].default_value = (*linear, 1)
        shader.inputs['Metallic'].default_value = .05
        shader.inputs['Roughness'].default_value = roughness
        return mat

    def box(name, size, center, mat, bevel):
        bpy.ops.mesh.primitive_cube_add(size=1, location=center)
        obj = bpy.context.object
        obj.name, obj.dimensions = name, size
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        obj.data.materials.append(mat)
        edge = obj.modifiers.new('Edge_highlights', 'BEVEL')
        edge.width, edge.segments = bevel, 3
        bpy.ops.object.modifier_apply(modifier=edge.name)
        obj.data.use_auto_smooth = True
        obj.modifiers.new('Surface_normals', 'WEIGHTED_NORMAL')
        return obj

    box('Backing', (.009, .264, .218), (.14, 0, 1.073),
        material('Backing_green', (.17, .34, .12), .4), .00065)
    colors = []
    for row in range(8):
        colors.append([])
        for col in range(8):
            rgb = sample(row, col)
            colors[-1].append([round(c * 255) for c in rgb])
            dark = max(rgb) < .30
            front, back = (.1493 if dark else .1512), .1443
            box(f'Pixel_{row}_{col}', (front - back, .264 / 8 - .00005, .218 / 8 - .00005),
                ((front + back) / 2, .132 - (col + .5) * .264 / 8,
                 1.182 - (row + .5) * .218 / 8),
                material(f'Pixel_{row}_{col}', rgb, .48 if dark else .34), .00016)

    COMPONENTS.mkdir(parents=True, exist_ok=True)
    meshes = PACKAGE / 'meshes'
    objects = sorted((obj for obj in scene.objects if obj.type == 'MESH'), key=lambda obj: obj.name)
    materials = [obj.data.materials[0] for obj in objects]
    usd = ['#usda 1.0', '(defaultPrim = "Component"\n metersPerUnit = 1\n upAxis = "Z")',
           'def Xform "Component" {', 'def Scope "Looks" {']
    mtl = []
    for mat in materials:
        shader = mat.node_tree.nodes.get('Principled BSDF')
        color = tuple(mat.diffuse_color[:3])
        roughness = shader.inputs['Roughness'].default_value
        usd += [f'def Material "{mat.name}" {{',
                f'token outputs:surface.connect = </Component/Looks/{mat.name}/Shader.outputs:surface>',
                'def Shader "Shader" { uniform token info:id = "UsdPreviewSurface"',
                f'color3f inputs:diffuseColor = {color}', 'float inputs:metallic = 0.05',
                f'float inputs:roughness = {roughness}', 'token outputs:surface\n}\n}']
        mtl += [f'newmtl {mat.name}', 'Kd ' + ' '.join(f'{c:.8f}' for c in mat['source_srgb']),
                'Ka 0.04 0.04 0.04', 'Ks 0.15 0.15 0.15', f'Ns {2 / roughness**2 - 2:.4f}', '']
    usd.append('}')
    obj_lines = [f'mtllib {NAME}.mtl']
    vertex_offset = normal_offset = 1
    depsgraph = bpy.context.evaluated_depsgraph_get()
    for obj in objects:
        evaluated = obj.evaluated_get(depsgraph)
        mesh = evaluated.to_mesh()
        mesh.calc_normals_split()
        points = [tuple(obj.matrix_world @ v.co) for v in mesh.vertices]
        counts = [len(face.vertices) for face in mesh.polygons]
        indices = [i for face in mesh.polygons for i in face.vertices]
        normals = [tuple(mesh.loops[i].normal) for face in mesh.polygons for i in face.loop_indices]
        usd += [f'def Mesh "{obj.name}" (prepend apiSchemas = ["MaterialBindingAPI"]) {{',
                f'point3f[] points = {points}', f'int[] faceVertexCounts = {counts}',
                f'int[] faceVertexIndices = {indices}',
                f'normal3f[] normals = {normals} (interpolation = "faceVarying")',
                'uniform token subdivisionScheme = "none"',
                f'rel material:binding = </Component/Looks/{obj.data.materials[0].name}>', '}']
        obj_lines += [f'o {obj.name}', f'usemtl {obj.data.materials[0].name}']
        obj_lines += ['v ' + ' '.join(f'{c:.9f}' for c in point) for point in points]
        obj_lines += ['vn ' + ' '.join(f'{c:.9f}' for c in normal) for normal in normals]
        corner = 0
        for face in mesh.polygons:
            obj_lines.append('f ' + ' '.join(f'{v + vertex_offset}//{normal_offset + corner + i}'
                                            for i, v in enumerate(face.vertices)))
            corner += len(face.vertices)
        vertex_offset += len(points)
        normal_offset += len(normals)
        evaluated.to_mesh_clear()
    usd.append('}')
    (COMPONENTS / f'{NAME}.usda').write_text('\n'.join(usd) + '\n')
    (meshes / f'{NAME}.obj').write_text('\n'.join(obj_lines) + '\n')
    (meshes / f'{NAME}.mtl').write_text('\n'.join(mtl))

    scene['reference'] = 'User-supplied Creeper_Head_(S)_JE1.png; front cell centers rectified to 8x8'
    scene['panel_bounds_m'] = [.1355, -.132, .964, .1512, .132, 1.182]
    scene['front_srgb_8x8'] = json.dumps(colors)
    scene.render.engine = 'CYCLES'
    scene.cycles.samples = 64
    scene.cycles.use_denoising = False
    scene.render.resolution_x, scene.render.resolution_y = 800, 650
    scene.render.resolution_percentage = 100
    scene.world.color = (.25, .25, .25)
    target = Vector((.14, 0, 1.073))
    bpy.ops.object.camera_add(location=(.7, -.22, 1.26))
    camera = bpy.context.object
    camera.rotation_euler = (target - camera.location).to_track_quat('-Z', 'Y').to_euler()
    camera.data.type, camera.data.ortho_scale = 'ORTHO', .38
    scene.camera = camera
    for location, energy, size in [((.6, -.3, 1.6), 35, .6), ((.4, .5, 1.2), 15, .5)]:
        bpy.ops.object.light_add(type='AREA', location=location)
        light = bpy.context.object
        light.data.energy, light.data.shape, light.data.size = energy, 'DISK', size
        light.rotation_euler = (target - light.location).to_track_quat('-Z', 'Y').to_euler()
    bpy.ops.wm.save_as_mainfile(filepath=str(COMPONENTS / f'{NAME}.blend'))
    preview = PROJECT / '.runtime/scout-creeper/blender-face.png'
    preview.parent.mkdir(parents=True, exist_ok=True)
    scene.render.filepath = str(preview)
    bpy.ops.render.render(write_still=True)


def compose_descriptions():
    from pxr import Gf, Usd, UsdGeom

    path = PROJECT / 'assets/robot/scout_creeper.usda'
    stage = Usd.Stage.CreateNew(str(path))
    robot = UsdGeom.Xform.Define(stage, '/Robot').GetPrim()
    stage.SetDefaultPrim(robot)
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    robot.GetReferences().AddReference('scout.usd')
    stage.OverridePrim('/Robot/base_link/visual/yellow').SetActive(False)
    face = UsdGeom.Xform.Define(stage, '/Robot/base_link/visual/creeper_face').GetPrim()
    face.GetReferences().AddReference(f'components/{NAME}.usda')

    # The screen mesh also owns the laptop display; remove only the old face inset.
    screen = UsdGeom.Mesh(stage.GetPrimAtPath('/Robot/base_link/visual/screen'))
    points, normals = screen.GetPointsAttr().Get(), screen.GetNormalsAttr().Get()
    indices = list(screen.GetFaceVertexIndicesAttr().Get())
    counts = list(screen.GetFaceVertexCountsAttr().Get())
    kept, offset, removed = [], 0, 0
    for count in counts:
        face_indices = indices[offset:offset + count]
        inside = [(.135 <= points[i][0] <= .153 and abs(points[i][1]) <= .133
                   and .963 <= points[i][2] <= 1.183) for i in face_indices]
        if any(inside) != all(inside):
            raise ValueError('Original face boundary intersects another screen surface')
        if not all(inside):
            kept.append(face_indices)
        else:
            removed += 1
        offset += count
    if removed != 208 or len(kept) != 208:
        raise ValueError('Source Scout face changed; review the screen extraction')
    used = sorted({i for polygon in kept for i in polygon})
    remap = {old: new for new, old in enumerate(used)}
    xyz, normal = [points[i] for i in used], [normals[i] for i in used]
    polygons = [[remap[i] for i in polygon] for polygon in kept]
    screen.GetPointsAttr().Set(xyz)
    screen.GetNormalsAttr().Set(normal)
    screen.GetFaceVertexCountsAttr().Set([len(polygon) for polygon in polygons])
    screen.GetFaceVertexIndicesAttr().Set([i for polygon in polygons for i in polygon])
    screen.GetExtentAttr().Set([Gf.Vec3f(*(min(p[a] for p in xyz) for a in range(3))),
                                Gf.Vec3f(*(max(p[a] for p in xyz) for a in range(3)))])
    stage.GetRootLayer().Save()
    obj = ['mtllib robot.mtl', 'usemtl screen']
    obj += ['v ' + ' '.join(f'{c:.9f}' for c in p) for p in xyz]
    obj += ['vn ' + ' '.join(f'{c:.9f}' for c in n) for n in normal]
    obj += ['f ' + ' '.join(f'{i + 1}//{i + 1}' for i in polygon) for polygon in polygons]
    (PACKAGE / 'meshes/base_link_screen_creeper.obj').write_text('\n'.join(obj) + '\n')

    description = (PACKAGE / 'urdf/scout_twin.urdf').read_text()
    description = description.replace('name="base_link_yellow"', 'name="base_link_creeper_face"')
    description = description.replace('meshes/base_link_yellow.obj', f'meshes/{NAME}.obj')
    description = description.replace('      <material name="yellow" />\n', '')
    description = description.replace('meshes/base_link_screen.obj', 'meshes/base_link_screen_creeper.obj')
    (PACKAGE / 'urdf/scout_twin_creeper.urdf').write_text(description)


if __name__ == '__main__':
    if '--blender' in sys.argv:
        model_in_blender()
    else:
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument('--reference', type=Path, default=REFERENCE)
        args = parser.parse_args()
        REFERENCE.parent.mkdir(parents=True, exist_ok=True)
        if args.reference.resolve() != REFERENCE.resolve():
            shutil.copyfile(args.reference, REFERENCE)
        subprocess.run(['blender', '--background', '--factory-startup', '--threads', '8',
                        '--python-exit-code', '1', '--python', str(Path(__file__).resolve()),
                        '--', '--blender'], check=True)
        compose_descriptions()
