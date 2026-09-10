#!/usr/bin/env python3
"""Offline structural/geometry checks using OpenUSD (no Isaac/PhysX execution).

Run: <Isaac Sim>/python.sh tests/validate_usd.py
or Python with usd-core installed. Exits nonzero if any check fails.
"""
import argparse
import json
import math
from pathlib import Path

from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdUtils

ROOT = Path(__file__).resolve().parents[1]
TOL = 1e-5


def close(a, b, tol=TOL):
    if isinstance(a, (tuple, list)) or isinstance(b, (tuple, list)):
        return len(a) == len(b) and all(abs(float(x)-float(y)) <= tol for x, y in zip(a, b))
    return abs(float(a)-float(b)) <= tol


def validate(project):
    cfg = json.loads((project / 'config/robot.json').read_text())
    checks, details = [], {}

    def check(name, condition, observed=None):
        item = {'name': name, 'passed': bool(condition)}
        if observed is not None:
            item['observed'] = observed
        checks.append(item)

    def body_mass(stage, root):
        return sum(float(UsdPhysics.MassAPI(p).GetMassAttr().Get())
                   for p in Usd.PrimRange(stage.GetPrimAtPath(root)) if p.HasAPI(UsdPhysics.RigidBodyAPI))

    stages = {}
    for file, root in [('robot.usd', '/Robot'), ('robot.usda', '/Robot'),
                       ('test_world.usd', '/World'), ('test_world_payload250.usd', '/World')]:
        path = project / 'assets' / file
        stage = Usd.Stage.Open(str(path))
        stages[file] = stage
        check(file + ': stage opens and default prim matches',
              stage is not None and str(stage.GetDefaultPrim().GetPath()) == root)
        if stage is None:
            continue
        check(file + ': units are metres/kilograms with Z up',
              close(UsdGeom.GetStageMetersPerUnit(stage), 1) and
              close(UsdPhysics.GetStageKilogramsPerUnit(stage), 1) and
              UsdGeom.GetStageUpAxis(stage) == 'Z')
        composition_errors = [str(error) for prim in [stage.GetPseudoRoot(), *stage.TraverseAll()]
                              for error in prim.GetPrimIndex().localErrors]
        check(file + ': no composition errors', not composition_errors, composition_errors)
        layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(path))
        check(file + ': all asset dependencies resolve', not unresolved, list(unresolved))
        check(file + ': dependencies are local and inside this package',
              all(Path(layer.realPath).resolve().is_relative_to(project.resolve()) for layer in layers) and
              all(Path(asset).resolve().is_relative_to(project.resolve()) for asset in assets))
        robot_root = '/Robot' if root == '/Robot' else '/World/Robot'
        mass = body_mass(stage, robot_root)
        expected = cfg['mass_kg'] + (cfg['max_payload_kg'] if 'payload250' in file else 0)
        check(file + ': rigid-link masses sum to requested mass', close(mass, expected),
              {'sum_kg': mass, 'expected_kg': expected})
        if root == '/World':
            scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
            check(file + ': exactly one authored physics scene', len(scenes) == 1,
                  [str(p.GetPath()) for p in scenes])
            check(file + ': downward Earth gravity', len(scenes) == 1 and
                  close(list(UsdPhysics.Scene(scenes[0]).GetGravityDirectionAttr().Get()), [0,0,-1]) and
                  close(UsdPhysics.Scene(scenes[0]).GetGravityMagnitudeAttr().Get(), 9.81))
            ground = stage.GetPrimAtPath('/World/Ground/CollisionPlane')
            ground_transform = UsdGeom.XformCache().GetLocalToWorldTransform(ground)
            check(file + ': static Z-up ground plane at z=0',
                  ground.IsA(UsdGeom.Plane) and ground.HasAPI(UsdPhysics.CollisionAPI) and
                  not ground.HasAPI(UsdPhysics.RigidBodyAPI) and ground.GetAttribute('axis').Get() == 'Z' and
                  close(ground_transform.ExtractTranslation()[2], 0))

    stage = stages['robot.usd']
    root = stage.GetDefaultPrim()
    bodies = {str(p.GetPath()): p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    joints = [UsdPhysics.RevoluteJoint(p) for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
    articulations = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    check('robot: one articulation root', len(articulations) == 1)
    check('robot: 11 rigid bodies (base, 2 drive wheels, 4 swivels, 4 caster wheels)', len(bodies) == 11,
          sorted(bodies))
    check('robot: 10 revolute joints', len(joints) == 10)
    nested = []
    for path, prim in bodies.items():
        ancestor = prim.GetParent()
        while ancestor and not ancestor.IsPseudoRoot():
            if ancestor.HasAPI(UsdPhysics.RigidBodyAPI):
                nested.append(path)
            ancestor = ancestor.GetParent()
    check('robot: no nested rigid-body prims', not nested, nested)
    check('robot: articulation self collisions explicitly disabled',
          root.GetAttribute('physxArticulation:enabledSelfCollisions').Get() is False)

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    driven, passive, anchors, parents = [], [], [], {}
    axes_ok, endpoints_ok = True, True
    for joint in joints:
        prim = joint.GetPrim()
        path = str(prim.GetPath())
        a, b = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
        endpoint_ok = len(a) == len(b) == 1 and str(a[0]) in bodies and str(b[0]) in bodies
        endpoints_ok &= endpoint_ok
        if not endpoint_ok:
            continue
        a, b = str(a[0]), str(b[0])
        parents.setdefault(b, []).append(a)
        m0, m1 = [cache.GetLocalToWorldTransform(bodies[v]) for v in (a,b)]
        p0, p1 = [Gf.Vec3d(v) for v in (joint.GetLocalPos0Attr().Get(), joint.GetLocalPos1Attr().Get())]
        gap = (m0.Transform(p0)-m1.Transform(p1)).GetLength()
        anchors.append({'joint': prim.GetName(), 'gap_m': gap})
        axis = {'X': Gf.Vec3d(1,0,0), 'Y': Gf.Vec3d(0,1,0), 'Z': Gf.Vec3d(0,0,1)}[joint.GetAxisAttr().Get()]
        q0, q1 = joint.GetLocalRot0Attr().Get(), joint.GetLocalRot1Attr().Get()
        a0 = m0.TransformDir(Gf.Rotation(Gf.Quatd(q0)).TransformDir(axis)).GetNormalized()
        a1 = m1.TransformDir(Gf.Rotation(Gf.Quatd(q1)).TransformDir(axis)).GetNormalized()
        axes_ok &= (a0-a1).GetLength() < TOL
        if prim.HasAPI(UsdPhysics.DriveAPI, 'angular'):
            driven.append(prim.GetName())
            drive = UsdPhysics.DriveAPI(prim, 'angular')
            check(prim.GetName() + ': force-limited pure velocity drive',
                  drive.GetTypeAttr().Get() == 'force' and close(drive.GetStiffnessAttr().Get(), 0) and
                  close(drive.GetDampingAttr().Get(), cfg['drive']['velocity_damping']) and
                  close(drive.GetMaxForceAttr().Get(), cfg['drive']['torque_limit_nm']) and
                  close(drive.GetTargetVelocityAttr().Get(), 0) and joint.GetAxisAttr().Get() == 'Y')
        else:
            passive.append(prim.GetName())
        check(prim.GetName() + ': no finite joint travel limits',
              not math.isfinite(joint.GetLowerLimitAttr().Get()) and
              not math.isfinite(joint.GetUpperLimitAttr().Get()))
    check('robot: joint endpoints reference rigid links', endpoints_ok)
    check('robot: all joint anchors coincide in world coordinates',
          len(anchors) == 10 and all(a['gap_m'] < TOL for a in anchors), anchors)
    check('robot: joint axes match across both body frames', axes_ok)
    check('robot: exactly two powered drive joints', sorted(driven) == ['left_drive_joint','right_drive_joint'], driven)
    check('robot: eight unactuated caster joints', len(passive) == 8, passive)
    tree_ok = '/Robot/base_link' not in parents and all(len(v) == 1 for v in parents.values())
    for path in bodies:
        visited, cursor = set(), path
        while cursor in parents and len(parents[cursor]) == 1 and cursor not in visited:
            visited.add(cursor)
            cursor = parents[cursor][0]
        tree_ok &= cursor == '/Robot/base_link'
    check('robot: connected acyclic articulation tree rooted at base_link', tree_ok)

    for path, prim in bodies.items():
        mass_api = UsdPhysics.MassAPI(prim)
        mass = float(mass_api.GetMassAttr().Get())
        inertia = list(mass_api.GetDiagonalInertiaAttr().Get())
        check(prim.GetName()+': finite positive mass and physically possible inertia',
              math.isfinite(mass) and mass > 0 and all(math.isfinite(x) and x > 0 for x in inertia) and
              all(inertia[i] <= sum(inertia)-inertia[i]+TOL for i in range(3)))

    bcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default','render'],
                              useExtentsHint=False, ignoreVisibility=False)
    bounds = bcache.ComputeWorldBound(root).ComputeAlignedRange()
    lo, hi = list(bounds.GetMin()), list(bounds.GetMax())
    size = [hi[i]-lo[i] for i in range(3)]
    desired = [cfg['dimensions'][k] for k in ('length','width','height')]
    check('robot: visible envelope exactly matches 800x580x245 mm specification', close(size, desired),
          {'min_m': lo, 'max_m': hi, 'size_m': size, 'tolerance_m': TOL})
    check('robot: visible envelope centred in XY with ground-contact bottom',
          close(lo, [-desired[0]/2,-desired[1]/2,0]) and
          close(hi, [desired[0]/2,desired[1]/2,desired[2]]))
    wheel_details = []
    for name in ['left_drive','right_drive'] + ['caster_'+k+'_wheel' for k in ['fl','fr','rl','rr']]:
        prim = stage.GetPrimAtPath('/Robot/'+name+'/Collision')
        geom = UsdGeom.Cylinder(prim)
        wheelcfg = cfg['drive'] if 'drive' in name else cfg['caster']
        center = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
        check(name+': correct cylindrical road collider tangent to ground',
              bool(geom) and prim.HasAPI(UsdPhysics.CollisionAPI) and geom.GetAxisAttr().Get() == 'Y' and
              close(geom.GetRadiusAttr().Get(), wheelcfg['radius_m']) and
              close(geom.GetHeightAttr().Get(), wheelcfg['width_m']) and
              close(center[2]-geom.GetRadiusAttr().Get(), 0))
        wheel_details.append({'link': name, 'centre_m': list(center), 'radius_m': geom.GetRadiusAttr().Get()})
    check('robot: drive wheel centre separation matches controller track',
          close(abs(wheel_details[0]['centre_m'][1]-wheel_details[1]['centre_m'][1]), cfg['drive']['track_m']))
    details['road_contact_wheels'] = wheel_details
    for key in ['fl','fr','rl','rr']:
        for part in ['Bearing','ForkL','ForkR']:
            prim = stage.GetPrimAtPath('/Robot/caster_'+key+'_swivel/'+part)
            check(key+'/'+part+': external collider enabled', prim.HasAPI(UsdPhysics.CollisionAPI) and
                  UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get())
    chassis_bounds = [bcache.ComputeWorldBound(p).ComputeAlignedRange()
                      for p in Usd.PrimRange(stage.GetPrimAtPath('/Robot/base_link/Collisions'))
                      if p.HasAPI(UsdPhysics.CollisionAPI)]
    def overlaps(a, b):
        return all(min(a.GetMax()[i], b.GetMax()[i])-max(a.GetMin()[i], b.GetMin()[i]) > TOL for i in range(3))
    for name in ['left_drive','right_drive']:
        wheel_bound = bcache.ComputeWorldBound(stage.GetPrimAtPath('/Robot/'+name+'/Collision')).ComputeAlignedRange()
        check(name+': chassis leaves the wheel clear', not any(overlaps(wheel_bound, b) for b in chassis_bounds))
    caster = cfg['caster']
    for key, sx, sy in [('fl',1,1),('fr',1,-1),('rl',-1,1),('rr',-1,-1)]:
        clear = True
        for degrees in range(0,360,5):
            yaw = math.radians(degrees)
            cx = sx*caster['pivot_x_m']-caster['trail_m']*math.cos(yaw)
            cy = sy*caster['pivot_y_m']-caster['trail_m']*math.sin(yaw)
            ex = caster['radius_m']*abs(math.cos(yaw))+caster['width_m']/2*abs(math.sin(yaw))
            ey = caster['radius_m']*abs(math.sin(yaw))+caster['width_m']/2*abs(math.cos(yaw))
            sweep = Gf.Range3d(Gf.Vec3d(cx-ex,cy-ey,0),Gf.Vec3d(cx+ex,cy+ey,2*caster['radius_m']))
            clear &= not any(overlaps(sweep, b) for b in chassis_bounds)
        check(key+': full caster swivel sweep clears chassis', clear)

    for side in ['front','rear']:
        link = stage.GetPrimAtPath('/Robot/base_link/'+side+'_camera_link')
        cam = UsdGeom.Camera(stage.GetPrimAtPath(str(link.GetPath())+'/camera'))
        matrix = cache.GetLocalToWorldTransform(cam.GetPrim())
        expected = cfg['camera'][side+'_pose_m_deg']
        yaw = math.radians(expected[3])
        fwd = [math.cos(yaw),math.sin(yaw),0]
        check(side+': camera position agrees with configuration', close(list(matrix.ExtractTranslation()), expected[:3]))
        check(side+': USD camera -Z points outward and +Y points up',
              close(list(matrix.TransformDir(Gf.Vec3d(0,0,-1))), fwd) and
              close(list(matrix.TransformDir(Gf.Vec3d(0,1,0))), [0,0,1]))
        hfov = math.degrees(2*math.atan(cam.GetHorizontalApertureAttr().Get()/(2*cam.GetFocalLengthAttr().Get())))
        aspect = cam.GetHorizontalApertureAttr().Get()/cam.GetVerticalApertureAttr().Get()
        check(side+': camera FOV and aspect match declared stream',
              close(hfov, cfg['camera']['hfov_deg'], 1e-4) and
              close(aspect, cfg['camera']['width']/cfg['camera']['height']))
        # Runtime optical frame rotates camera local X by pi: +Z optical is forward, +Y down.
        optical = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1,0,0),180)) * matrix
        check(side+': ROS optical conversion has +Z forward and +Y down',
              close(list(optical.TransformDir(Gf.Vec3d(0,0,1))), fwd) and
              close(list(optical.TransformDir(Gf.Vec3d(0,1,0))), [0,0,-1]))
    for side in ['front_right','rear_left']:
        matrix = cache.GetLocalToWorldTransform(stage.GetPrimAtPath('/Robot/base_link/'+side+'_lidar_link'))
        expected = cfg['lidar'][side+'_pose_m_deg']
        yaw = math.radians(expected[3])
        check(side+': LiDAR position and outward yaw agree with configuration',
              close(list(matrix.ExtractTranslation()), expected[:3]) and
              close(list(matrix.TransformDir(Gf.Vec3d(1,0,0))), [math.cos(yaw),math.sin(yaw),0]))

    loaded = stages['test_world_payload250.usd']
    base_mass = UsdPhysics.MassAPI(loaded.GetPrimAtPath('/World/Robot/base_link'))
    m0, mp = cfg['base_mass_kg'], cfg['max_payload_kg']
    offset = cfg['dimensions']['height'] + cfg['payload']['size_m'][2]/2 - cfg['base_link_z_m']
    expected_com = mp*offset/(m0+mp)
    check('payload: composite base mass and centre of mass are correct',
          close(base_mass.GetMassAttr().Get(), m0+mp) and
          close(list(base_mass.GetCenterOfMassAttr().Get()), [0,0,expected_com]))
    loaded_bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default','render'], False, True)
    payload = loaded.GetPrimAtPath('/World/Robot/base_link/Payload')
    check('payload: fixed geometry bottom touches deck top without penetration',
          payload.HasAPI(UsdPhysics.CollisionAPI) and not payload.HasAPI(UsdPhysics.RigidBodyAPI) and
          close(loaded_bounds.ComputeWorldBound(payload).ComputeAlignedRange().GetMin()[2], cfg['dimensions']['height']))
    result = {
        'scope': 'OpenUSD offline structure, geometry, units, mass, joint frames, sensor axes and dependencies',
        'usd_validator_version': '.'.join(str(x) for x in Usd.GetVersion()),
        'isaac_sim_executed': False,
        'physx_or_rtx_execution_verified': False,
        'ros_dds_streams_verified': False,
        'note': 'Raw PhysxSchema property tokens are inspected as authored USD; this does not load or validate NVIDIA PhysX behavior.',
        'passed': all(c['passed'] for c in checks),
        'checks_passed': sum(c['passed'] for c in checks),
        'checks_total': len(checks),
        'checks': checks,
        'details': details,
    }
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project', type=Path, default=ROOT)
    p.add_argument('--report', type=Path, default=ROOT/'verification/usd_validation.json')
    args = p.parse_args()
    report = validate(args.project.resolve())
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    print(f"USD offline validation: {report['checks_passed']}/{report['checks_total']} passed")
    for check in report['checks']:
        if not check['passed']:
            print('FAIL:', check['name'], check.get('observed',''))
    print('Report:', args.report)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
