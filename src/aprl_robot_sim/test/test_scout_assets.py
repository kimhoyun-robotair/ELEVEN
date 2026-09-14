from pathlib import Path
from itertools import product
import unittest
import xml.etree.ElementTree as ET

try:
    from pxr import Usd, UsdGeom, UsdPhysics
except ImportError:
    Usd = None


PROJECT = Path(__file__).resolve().parents[3]
DESCRIPTION = PROJECT / 'src/scout_twin_description/urdf/scout_twin.urdf'
CREEPER_DESCRIPTION = DESCRIPTION.with_name('scout_twin_creeper.urdf')


class ScoutDescriptionTests(unittest.TestCase):
    def test_every_mesh_and_material_is_portable(self):
        for description in (DESCRIPTION, CREEPER_DESCRIPTION):
            with self.subTest(description=description.name):
                robot = ET.parse(description).getroot()
                self.assertEqual(robot.get('name'), 'scout_mini_photo_twin')
                for mesh in robot.findall('.//mesh'):
                    uri = mesh.attrib['filename']
                    self.assertTrue(uri.startswith('package://scout_twin_description/meshes/'))
                    path = PROJECT / 'src' / uri.removeprefix('package://')
                    self.assertTrue(path.is_file(), uri)
                    for line in path.read_text().splitlines():
                        if line.startswith('mtllib '):
                            self.assertTrue((path.parent / line.split(maxsplit=1)[1]).is_file(), line)

    def test_creeper_changes_only_the_two_face_visuals(self):
        original, creeper = (ET.parse(path).getroot() for path in (DESCRIPTION, CREEPER_DESCRIPTION))
        for robot in (original, creeper):
            base = robot.find("link[@name='base_link']")
            assert base is not None
            for visual in list(base.findall('visual')):
                if visual.get('name') in ('base_link_yellow', 'base_link_creeper_face', 'base_link_screen'):
                    base.remove(visual)
        self.assertEqual(ET.tostring(original), ET.tostring(creeper))


@unittest.skipIf(Usd is None, 'OpenUSD Python bindings required for USD asset checks')
class ScoutUsdTests(unittest.TestCase):
    def test_creeper_preserves_every_original_prim_outside_the_face(self):
        original = Usd.Stage.Open(str(PROJECT / 'assets/robot/scout.usd'))
        creeper = Usd.Stage.Open(str(PROJECT / 'assets/robot/scout_creeper.usda'))
        allowed = {'/Robot/base_link/visual/yellow', '/Robot/base_link/visual/screen'}
        for prim in original.Traverse():
            if str(prim.GetPath()) in allowed:
                continue
            actual = creeper.GetPrimAtPath(prim.GetPath())
            with self.subTest(prim=str(prim.GetPath())):
                self.assertTrue(actual)
                self.assertEqual(actual.GetTypeName(), prim.GetTypeName())
                self.assertEqual(actual.GetAppliedSchemas(), prim.GetAppliedSchemas())
                for attr in prim.GetAttributes():
                    self.assertEqual(actual.GetAttribute(attr.GetName()).Get(), attr.Get())
                for relation in prim.GetRelationships():
                    self.assertEqual(actual.GetRelationship(relation.GetName()).GetTargets(), relation.GetTargets())
        for prim in creeper.Traverse():
            if not original.GetPrimAtPath(prim.GetPath()):
                self.assertFalse(prim.HasAPI(UsdPhysics.CollisionAPI))
                self.assertFalse(prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(creeper.GetPrimAtPath('/Robot/base_link/visual/yellow').IsActive())

    def test_creeper_matches_the_original_face_envelope_in_usd_and_mesh(self):
        bounds = []
        for filename, path in (('scout.usd', '/Robot/base_link/visual/yellow'),
                               ('scout_creeper.usda', '/Robot/base_link/visual/creeper_face')):
            stage = Usd.Stage.Open(str(PROJECT / 'assets/robot' / filename))
            box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default']).ComputeWorldBound(
                stage.GetPrimAtPath(path)).ComputeAlignedRange()
            bounds.append([*box.GetMin(), *box.GetMax()])
        mesh = PROJECT / 'src/scout_twin_description/meshes/scout_creeper_face.obj'
        points = [list(map(float, line.split()[1:])) for line in mesh.read_text().splitlines()
                  if line.startswith('v ')]
        bounds.append([*(min(p[a] for p in points) for a in range(3)),
                       *(max(p[a] for p in points) for a in range(3))])
        for bound in bounds:
            for value, expected in zip(bound, [.1355, -.132, .964, .1512, .132, 1.182]):
                self.assertAlmostEqual(value, expected, places=6)

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
        for scene, asset in product(('office', 'house'), ('scout.usd', 'scout_creeper.usda')):
            with self.subTest(scene=scene, asset=asset):
                stage = Usd.Stage.Open(str(PROJECT / f'assets/scenes/{scene}.usda'))
                stage.SetEditTarget(stage.GetSessionLayer())
                prim = UsdGeom.Xform.Define(stage, '/World/Robot')
                prim.GetPrim().GetReferences().AddReference(str(PROJECT / 'assets/robot' / asset))
                self.assertTrue(stage.GetPrimAtPath('/World/Robot/base_link/mid360_link/imu_link'))
                self.assertTrue(stage.GetPrimAtPath('/World/Elevators'))
                for layer in stage.GetUsedLayers():
                    if not layer.anonymous:
                        self.assertTrue(Path(layer.realPath).is_relative_to(PROJECT))


if __name__ == '__main__':
    unittest.main()
