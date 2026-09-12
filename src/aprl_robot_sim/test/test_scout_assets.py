from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

try:
    from pxr import Usd, UsdGeom, UsdPhysics
except ImportError:
    Usd = None


PROJECT = Path(__file__).resolve().parents[3]
DESCRIPTION = PROJECT / 'src/scout_twin_description/urdf/scout_twin.urdf'


class ScoutDescriptionTests(unittest.TestCase):
    def test_every_mesh_and_material_is_portable(self):
        robot = ET.parse(DESCRIPTION).getroot()
        self.assertEqual(robot.get('name'), 'scout_mini_photo_twin')
        for mesh in robot.findall('.//mesh'):
            uri = mesh.attrib['filename']
            self.assertTrue(uri.startswith('package://scout_twin_description/meshes/'))
            path = PROJECT / 'src' / uri.removeprefix('package://')
            self.assertTrue(path.is_file(), uri)
            for line in path.read_text().splitlines():
                if line.startswith('mtllib '):
                    self.assertTrue((path.parent / line.split(maxsplit=1)[1]).is_file(), line)


@unittest.skipIf(Usd is None, 'OpenUSD Python bindings required for USD asset checks')
class ScoutUsdTests(unittest.TestCase):
    def test_usd_and_urdf_agree_on_drive_joints(self):
        stage = Usd.Stage.Open(str(PROJECT / 'assets/robot/scout.usd'))
        self.assertTrue(stage.GetPrimAtPath('/Robot/base_link').HasAPI(UsdPhysics.ArticulationRootAPI))
        expected = {j.attrib['name']: j for j in ET.parse(DESCRIPTION).getroot().findall('joint')
                    if j.get('type') == 'continuous'}
        actual = [p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
        self.assertEqual({p.GetName() for p in actual}, set(expected))
        self.assertEqual(len(actual), 4)
        cache = UsdGeom.XformCache()
        for prim in actual:
            joint = UsdPhysics.RevoluteJoint(prim)
            self.assertEqual(joint.GetAxisAttr().Get(), 'Y')
            xml = expected[prim.GetName()]
            wheel = stage.GetPrimAtPath(str(joint.GetBody1Rel().GetTargets()[0]))
            self.assertEqual(wheel.GetName(), xml.find('child').attrib['link'])
            position = cache.GetLocalToWorldTransform(wheel).ExtractTranslation()
            reference = list(map(float, xml.find('origin').attrib['xyz'].split()))
            for actual_value, expected_value in zip(position, reference):
                self.assertAlmostEqual(actual_value, expected_value, places=6)

    def test_scout_composes_in_both_aprl_worlds(self):
        for scene in ('office', 'house'):
            with self.subTest(scene=scene):
                stage = Usd.Stage.Open(str(PROJECT / f'assets/scenes/{scene}.usda'))
                stage.SetEditTarget(stage.GetSessionLayer())
                prim = UsdGeom.Xform.Define(stage, '/World/Robot')
                prim.GetPrim().GetReferences().AddReference(str(PROJECT / 'assets/robot/scout.usd'))
                self.assertTrue(stage.GetPrimAtPath('/World/Robot/base_link/mid360_link/imu_link'))
                self.assertTrue(stage.GetPrimAtPath('/World/Elevators'))
                for layer in stage.GetUsedLayers():
                    if not layer.anonymous:
                        self.assertTrue(Path(layer.realPath).is_relative_to(PROJECT))


if __name__ == '__main__':
    unittest.main()
