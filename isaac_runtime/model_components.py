"""Rebuild the riser and wrist camera with Blender 4: blender -b -P this_file."""
import math
from pathlib import Path

import bpy

OUT = Path(__file__).resolve().parents[1] / 'assets/robot/components'


def material(name, color, metallic=0.0, roughness=.3):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*color, 1)
    mat.use_nodes = True
    shader = mat.node_tree.nodes.get('Principled BSDF')
    shader.inputs['Base Color'].default_value = (*color, 1)
    shader.inputs['Metallic'].default_value = metallic
    shader.inputs['Roughness'].default_value = roughness
    return mat


def box(name, size, center, mat, bevel=.001):
    bpy.ops.mesh.primitive_cube_add(size=1, location=center)
    obj = bpy.context.object
    obj.name, obj.dimensions = name, size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    obj.data.materials.append(mat)
    if bevel:
        mod = obj.modifiers.new('Machined_edges', 'BEVEL')
        mod.width, mod.segments = bevel, 3
        bpy.ops.object.modifier_apply(modifier=mod.name)
        obj.data.use_auto_smooth = True
        obj.modifiers.new('Face_normals', 'WEIGHTED_NORMAL')
    return obj


def cylinder(name, radius, depth, center, mat, rotation=(0, 0, 0)):
    bpy.ops.mesh.primitive_cylinder_add(vertices=48, radius=radius, depth=depth,
                                      location=center, rotation=rotation)
    obj = bpy.context.object
    obj.name = name
    obj.data.materials.append(mat)
    return obj


def export(name):
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT / f'{name}.blend'))
    asset_name = name
    lines = ['#usda 1.0', '(\n defaultPrim = "Component"\n metersPerUnit = 1\n upAxis = "Z"\n)',
             'def Xform "Component" {', 'def Scope "Looks" {']
    for mat in bpy.data.materials:
        if not mat.use_nodes or not mat.node_tree.nodes.get('Principled BSDF'):
            continue
        shader = mat.node_tree.nodes.get('Principled BSDF')
        color = tuple(shader.inputs['Base Color'].default_value[:3])
        lines += [f'def Material "{mat.name}" {{',
                  f'token outputs:surface.connect = </Component/Looks/{mat.name}/Shader.outputs:surface>',
                  'def Shader "Shader" {\n uniform token info:id = "UsdPreviewSurface"',
                  f'color3f inputs:diffuseColor = {color}',
                  f"float inputs:metallic = {shader.inputs['Metallic'].default_value}",
                  f"float inputs:roughness = {shader.inputs['Roughness'].default_value}",
                  'token outputs:surface\n }\n }']
    lines.append('}')
    graph = bpy.context.evaluated_depsgraph_get()
    for obj in bpy.context.scene.objects:
        evaluated = obj.evaluated_get(graph)
        mesh = evaluated.to_mesh()
        name = obj.name.replace('.', '_')
        points = [tuple(obj.matrix_world @ v.co) for v in mesh.vertices]
        counts = [len(face.vertices) for face in mesh.polygons]
        indices = [v for face in mesh.polygons for v in face.vertices]
        lines += [f'def Mesh "{name}" (prepend apiSchemas = ["MaterialBindingAPI"]) {{',
                  f'point3f[] points = {str(points)}', f'int[] faceVertexCounts = {counts}',
                  f'int[] faceVertexIndices = {indices}', 'uniform token subdivisionScheme = "none"',
                  f'rel material:binding = </Component/Looks/{obj.data.materials[0].name}>', '}']
        evaluated.to_mesh_clear()
    lines.append('}')
    (OUT / f'{asset_name}.usda').write_text('\n'.join(lines))



def main():
    OUT.mkdir(parents=True, exist_ok=True)
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    bpy.context.scene.unit_settings.system = 'METRIC'
    aluminum = material('Anodized_aluminum', (.52, .56, .60), .85)
    black = material('Slot_and_fastener', (.018, .022, .028), .4)
    # Four 30 mm T-slot columns, perimeter rails and bolted 8 mm plates.
    box('Base_plate', (.36, .32, .008), (0, 0, .004), aluminum)
    box('Arm_plate', (.32, .28, .008), (0, 0, .546), aluminum)
    for x in (-.135, .135):
        for y in (-.115, .115):
            box('Extrusion_column', (.03, .03, .534), (x, y, .275), aluminum)
            for sign in (-1, 1):
                box('T_slot', (.006, .0008, .51), (x, y+sign*.0151, .275), black, .0003)
                box('T_slot', (.0008, .006, .51), (x+sign*.0151, y, .275), black, .0003)
            for z in (.009, .541):
                cylinder('Socket_bolt', .0045, .003, (x, y, z), black)
                box('Gusset_bracket', (.045, .035, .045), (x, y, z+(.022 if z < .1 else -.022)), aluminum)
    for z in (.035, .515):
        for y in (-.115, .115):
            box('Cross_rail', (.24, .03, .03), (0, y, z), aluminum)
            box('Cross_slot', (.22, .0008, .006), (0, y+math.copysign(.0151, y), z), black)
        for x in (-.135, .135):
            box('Side_rail', (.03, .20, .03), (x, 0, z), aluminum)
    for x, y in ((-.032, -.032), (-.032, .032), (.032, -.032), (.032, .032)):
        cylinder('Arm_mount_bolt', .004, .004, (x, y, .552), black)
    export('riser')

    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    silver = material('Gemini_silver', (.55, .57, .59), .75, .25)
    glass = material('Gemini_black_glass', (.006, .008, .011), .15, .12)
    lens = material('Optical_coating', (.008, .030, .025), .4, .09)
    # Optical direction is +X; outside envelope is exactly 23 x 42 x 42 mm.
    box('Gemini_305_body', (.022, .042, .042), (-.012, 0, 0), silver, .0038)
    box('Front_bezel', (.0004, .040, .040), (-.0008, 0, 0), silver, .003)
    box('Front_glass', (.0002, .037, .037), (-.0005, 0, 0), glass, .0025)
    for y in (-.009, .009):
        for name, radius, x, mat in [('Lens_ring', .007, -.00036, silver),
                                     ('Lens_barrel', .0064, -.00024, black),
                                     ('Lens_glass', .0045, -.00012, lens),
                                     ('Lens_aperture', .0022, -.00004, glass)]:
            cylinder(name, radius, .00008, (x, y, 0), mat, (0, math.pi/2, 0))
    box('USB_C_recess', (.009, .0002, .004), (-.011, -.0209, 0), black, .001)
    for z in (-.012, .012):
        cylinder('USB_screw', .0012, .0002, (-.011, -.0209, z), black, (math.pi/2, 0, 0))
    export('gemini305')


if __name__ == '__main__':
    main()
