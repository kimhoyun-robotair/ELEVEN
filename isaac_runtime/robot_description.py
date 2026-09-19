"""Export the runtime robots and sensor frames as portable ROS descriptions."""
from copy import deepcopy
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET


def export_descriptions(root: Path):
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension('isaacsim.asset.exporter.urdf')
    from nvidia.srl.from_usd.to_urdf import UsdToUrdf
    from nvidia.srl.urdf_xml.schema import Color, Material
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    import trimesh

    class ColoredExporter(UsdToUrdf):
        # The 5.1 exporter preserves mesh materials but omits primitive displayColor.
        def _add_geometry(self, node):
            link = self.robot_links[node.to_neighbors[0].sku]
            count = len(link.visuals)
            super()._add_geometry(node)
            if not node.prim.IsA(UsdGeom.Mesh):
                colors = UsdGeom.Gprim(node.prim).GetDisplayColorAttr().Get()
                if colors:
                    for visual in link.visuals[count:]:
                        visual.material = Material(node.name, Color([*colors[0], 1.0]))

    def fixed(stage, parent, child, name=None):
        cache = UsdGeom.XformCache()
        matrix = cache.GetLocalToWorldTransform(child) * cache.GetLocalToWorldTransform(parent).GetInverse()
        joint = UsdPhysics.FixedJoint.Define(stage, '/Robot/' + (name or child.GetName() + '_joint'))
        joint.CreateBody0Rel().SetTargets([parent.GetPath()])
        joint.CreateBody1Rel().SetTargets([child.GetPath()])
        joint.CreateLocalPos0Attr(Gf.Vec3f(matrix.ExtractTranslation()))
        joint.CreateLocalRot0Attr(Gf.Quatf(matrix.ExtractRotationQuat()))

    package = root / 'src/aprl_robot_sim'
    nero = ET.parse(root / 'src/nero_description/urdf/nero_with_gripper_description.urdf').getroot()
    (package / 'urdf').mkdir(exist_ok=True)
    (package / 'meshes').mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='aprl-urdf-') as directory:
        temporary = Path(directory)
        for model in ('amr', 'locomanipulator'):
            stage = Usd.Stage.Open(str(root / f'assets/robot/{model}.usda'))
            stage.SetEditTarget(stage.GetSessionLayer())
            base = stage.GetPrimAtPath('/Robot/base_link')
            fixed(stage, stage.GetDefaultPrim(), base, 'base_joint')
            UsdGeom.Xform.Define(stage, base.GetPath().AppendChild('imu_link'))
            sensors = [prim for prim in stage.Traverse() if prim.GetName().endswith(('_camera_link', '_lidar_link'))
                       or prim.GetName() == 'imu_link']
            for sensor in sensors:
                fixed(stage, sensor.GetParent(), sensor)
                camera = stage.GetPrimAtPath(sensor.GetPath().AppendChild('camera'))
                if camera:
                    optical = UsdGeom.Xform.Define(stage, sensor.GetPath().AppendChild(
                        sensor.GetName().replace('_link', '_optical_frame')))
                    flip = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180))
                    optical.MakeMatrixXform().Set(flip * UsdGeom.Xformable(camera).GetLocalTransformation())
                    fixed(stage, sensor, optical.GetPrim())
                    camera.SetActive(False)
            for prim in stage.Traverse():
                if prim.IsA(UsdGeom.Scope):
                    prim.SetTypeName('Xform')
                if prim.IsA(UsdGeom.Gprim) and UsdGeom.Imageable(prim).ComputeVisibility() == 'invisible':
                    prim.CreateAttribute('purpose', prim.GetAttribute('purpose').GetTypeName()).Set('proxy')
            converter = ColoredExporter(stage, root='/Robot', log_level='ERROR')
            path = temporary / f'{model}.urdf'
            converter.save_to_file(path, mesh_dir=temporary / 'meshes', mesh_path_prefix='package://aprl_robot_sim/')
            robot = ET.parse(path).getroot()
            robot.set('name', model)
            for element in robot.iter():
                for key in ('name', 'link'):
                    value = element.get(key)
                    if value in ('Robot', 'Robot_base_link'):
                        element.set(key, {'Robot': 'base_footprint', 'Robot_base_link': 'base_link'}[value])
            # Keep the supplied link frames, including the second finger's negative joint axis.
            for joint in robot.findall('joint'):
                name = joint.attrib['name']
                source = nero.find(f"joint[@name='{name.removeprefix('nero_')}']") if name.startswith('nero_') else None
                if source is not None:
                    for tag in ('origin', 'axis'):
                        old, original = joint.find(tag), source.find(tag)
                        if old is not None:
                            joint.remove(old)
                        if original is not None:
                            joint.append(deepcopy(original))
            for link in robot.findall('link'):
                name = link.attrib['name']
                if not name.startswith('nero_'):
                    continue
                source = nero.find(f"link[@name='{name.removeprefix('nero_')}']")
                if source is None:
                    continue
                for kind in ('visual', 'collision'):
                    for geometry in link.findall(kind):
                        mesh = geometry.find('geometry/mesh')
                        if mesh is not None and Path(mesh.attrib['filename']).name.startswith(name + '_'):
                            link.remove(geometry)
                    link.extend(deepcopy(source.findall(kind)))
            # Jazzy RViz uses the first visual's URDF material; embed each primitive's color in OBJ.
            for visual in robot.findall('.//visual'):
                geometry = visual.find('geometry')
                shape = geometry[0]
                if shape.tag == 'mesh':
                    continue
                material = visual.find('material')
                color = material.find('color').attrib['rgba'].split()
                name = material.attrib['name']
                if shape.tag == 'box':
                    mesh = trimesh.creation.box(extents=list(map(float, shape.attrib['size'].split())))
                elif shape.tag == 'cylinder':
                    mesh = trimesh.creation.cylinder(radius=float(shape.attrib['radius']), height=float(shape.attrib['length']), sections=32)
                else:
                    mesh = trimesh.creation.icosphere(subdivisions=2, radius=float(shape.attrib['radius']))
                target = temporary / 'meshes' / name
                target.with_suffix('.obj').write_text(f'mtllib {name}.mtl\nusemtl surface\n' +
                    mesh.export(file_type='obj', include_color=False, include_texture=False, include_normals=True))
                target.with_suffix('.mtl').write_text(f'newmtl surface\nKd {" ".join(color[:3])}\nKa 1 1 1\nd {color[3]}\n')
                geometry.remove(shape)
                ET.SubElement(geometry, 'mesh', filename=f'package://aprl_robot_sim/meshes/{name}.obj')
                visual.remove(material)
            expected = {prim.GetName() for prim in stage.Traverse()
                        if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)}
            actual = {joint.get('name') for joint in robot.findall('joint') if joint.get('type') != 'fixed'}
            if actual != expected:
                raise RuntimeError(f'{model}: URDF joints differ from the USD articulation')
            for mesh in robot.findall('.//mesh'):
                if mesh.attrib['filename'].startswith('package://aprl_robot_sim/'):
                    source = temporary / 'meshes' / Path(mesh.attrib['filename']).name
                    for asset in (source, source.with_suffix('.mtl')):
                        if asset.exists():
                            shutil.copyfile(asset, package / 'meshes' / asset.name)
            ET.indent(robot, space='  ')
            ET.ElementTree(robot).write(package / 'urdf' / path.name, encoding='utf-8', xml_declaration=True)
            print(f'Exported {model}: {len(robot.findall("link"))} links, {len(actual)} movable joints', flush=True)


if __name__ == '__main__':
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'extra_args': ['--/app/settings/persistent=false']})
    code = 0
    try:
        export_descriptions(Path(__file__).resolve().parents[1])
    except Exception:
        import traceback
        traceback.print_exc()
        code = 1
    finally:
        app.app.post_quit(code)
        app.close()
