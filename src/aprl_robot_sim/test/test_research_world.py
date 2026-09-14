import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

from isaac_runtime.simulation import arguments

try:
    from pxr import Usd, UsdGeom, UsdPhysics, UsdUtils
    from isaac_runtime.elevator_usd import ElevatorScene
except ImportError:
    Usd = None


PROJECT = Path(__file__).resolve().parents[3]
SCENES = PROJECT / 'assets/scenes'
EXAMPLE = PROJECT / 'src/aprl_robot_sim/examples/research.json'
HEIGHTS = [0, 4, 8, 12, 16, 20]
LABELS = ['B1', '1F', '2F', '3F', '4F', 'RF']


class ResearchSelectionTests(unittest.TestCase):
    def test_research_is_available_for_every_robot(self):
        for robot in ('amr', 'locomanipulator', 'scout'):
            with self.subTest(robot=robot), patch.object(
                    sys, 'argv', ['simulation.py', '--scene', 'research', '--robot', robot]):
                args, _ = arguments()
                self.assertEqual(args.world, SCENES / 'research.usda')
                self.assertEqual(args.robot, robot)

    def test_example_travels_from_b1_to_1f_with_long_corridor_segments(self):
        route = json.loads(EXAMPLE.read_text())
        self.assertEqual(route['scene'], 'research')
        self.assertEqual(route['elevator'], 'E1')
        self.assertEqual((route['from_floor'], route['to_floor']), (1, 2))
        self.assertAlmostEqual(route['spawn'][2], .025)
        for start, points in ((route['spawn'][:2], route['before']),
                              (route['exit'], route['after'])):
            path = [start, *points]
            self.assertGreaterEqual(sum(math.dist(a, b) for a, b in zip(path, path[1:])), 6)


@unittest.skipIf(Usd is None, 'OpenUSD Python bindings required for USD asset checks')
class ResearchUsdTests(unittest.TestCase):
    def test_all_dependencies_are_local_except_isaac_builtin_glass(self):
        layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(SCENES / 'research.usda'))
        self.assertLessEqual(set(unresolved), {'OmniGlass.mdl'})
        self.assertTrue(layers)
        self.assertTrue(assets)
        for path in [*(layer.realPath for layer in layers), *assets]:
            with self.subTest(asset=path):
                self.assertTrue(Path(path).is_file())
                self.assertTrue(Path(path).resolve().is_relative_to(SCENES.resolve()))

    def test_three_physical_elevators_serve_all_six_labeled_levels(self):
        stage = Usd.Stage.Open(str(SCENES / 'research.usda'))
        scene = ElevatorScene(stage)
        config = json.loads((PROJECT / 'config/research.json').read_text())
        self.assertEqual(config['floorHeights'], HEIGHTS)
        self.assertEqual(config['floorLabels'], LABELS)
        self.assertEqual(config['footprint'], [60, 18])
        self.assertEqual(set(scene.rigs), {'E1', 'E2', 'E3'})
        self.assertEqual(scene.validate_physics(), [])
        for spec in config['elevators']:
            rig = scene.rigs[spec['id']]
            self.assertEqual(rig.heights, HEIGHTS)
            self.assertEqual(rig.floor_labels, LABELS)
            origin = UsdGeom.Xformable(rig.root).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            for actual, expected in zip(origin.ExtractTranslation(), spec['position']):
                self.assertAlmostEqual(actual, expected, places=5)
            for floor in range(6):
                with self.subTest(elevator=spec['id'], floor=LABELS[floor]):
                    self.assertTrue(scene.dispatch_press(f'{rig.path}/Cabin/Panel/Floor_{floor}'))
                    for _ in range(400):
                        state = rig.update(.05)
                        if state['floor'] == floor and state['state'] == 'dwell':
                            break
                    else:
                        self.fail(f'{rig.path} did not open at {LABELS[floor]}')
                    transform = UsdGeom.Xformable(rig.cabin.prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                    self.assertAlmostEqual(transform.ExtractTranslation()[2], HEIGHTS[floor])
                    self.assertFalse(any(state['cabinLights']))
                    for landing, doors in enumerate(rig.landing_doors):
                        self.assertAlmostEqual(doors[0].op.Get()[0], -.72 if landing == floor else 0)
                        self.assertAlmostEqual(doors[1].op.Get()[0], .72 if landing == floor else 0)

    def test_route_corridors_clear_the_tall_robot_envelope(self):
        stage = Usd.Stage.Open(str(SCENES / 'research.usda'))
        bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render'])
        route = json.loads(EXAMPLE.read_text())
        segments = [(0, route['spawn'][:2], route['before'][0]),
                    (4, route['exit'], route['after'][0])]
        for prim in stage.Traverse():
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            box = bounds.ComputeWorldBound(prim).ComputeAlignedBox()
            if box.IsEmpty():
                continue
            lo, hi = box.GetMin(), box.GetMax()
            for height, start, end in segments:
                overlaps = (lo[2] < height + 1.65 and hi[2] > height + .04
                            and lo[0] < max(start[0], end[0]) + .55
                            and hi[0] > min(start[0], end[0]) - .55
                            and lo[1] < max(start[1], end[1]) + .55
                            and hi[1] > min(start[1], end[1]) - .55)
                self.assertFalse(overlaps, str(prim.GetPath()))

    def test_existing_worlds_keep_numeric_floor_labels(self):
        for world, labels in (('office', ['1', '2', '3', '4']), ('house', ['1', '2', '3'])):
            stage = Usd.Stage.Open(str(SCENES / f'{world}.usda'))
            for rig in ElevatorScene(stage).rigs.values():
                self.assertEqual(rig.floor_labels, labels)


if __name__ == '__main__':
    unittest.main()
