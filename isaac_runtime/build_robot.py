"""Rebuild runtime USD assets from the supplied URDF and Blender components."""
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from isaacsim import SimulationApp

ROOT = Path(__file__).resolve().parents[1]
exit_code = 0
app = SimulationApp({'headless': True, 'extra_args': ['--/app/settings/persistent=false']})
try:
    import omni.kit.commands
    from isaacsim.core.utils.extensions import enable_extension
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema
    enable_extension('isaacsim.asset.importer.urdf')
    app.update()
    source = ROOT / 'src/nero_description'
    tree = ET.parse(source / 'urdf/nero_with_gripper_description.urdf')
    urdf = tree.getroot()
    for item in list(urdf):
        if item.get('name') in ('world', 'world_to_base_link', 'gripper', 'gripper_link'):
            urdf.remove(item)
    for item in urdf:
        item.set('name', 'nero_' + item.get('name'))
    for element in urdf.iter():
        for key in ('link', 'joint'):
            if element.get(key):
                element.set(key, 'nero_' + element.get(key))
        if element.tag == 'mesh':
            element.set('filename', str(source / element.get('filename').removeprefix('package://nero_description/')))
    urdf.find("link[@name='nero_link7']/inertial/mass").set('value', '0.03')
    for joint in urdf.findall('joint'):
        mimic = joint.find('mimic')
        if mimic is not None:
            joint.remove(mimic)
    with tempfile.TemporaryDirectory(prefix='nero-import-') as tmp:
        path = Path(tmp) / 'nero.urdf'
        tree.write(path)
        _, cfg = omni.kit.commands.execute('URDFCreateImportConfig')
        cfg.merge_fixed_joints = True
        cfg.fix_base = False
        cfg.import_inertia_tensor = True
        cfg.convex_decomp = True
        cfg.parse_mimic = False
        cfg.create_physics_scene = False
        ok, _ = omni.kit.commands.execute('URDFParseAndImportFile', urdf_path=str(path),
                                          import_config=cfg, dest_path=str(Path(tmp) / 'nero.usd'))
        if not ok:
            raise RuntimeError('NERO URDF import failed')
        arm = Usd.Stage.Open(str(Path(tmp) / 'nero.usd'))
        for prim in arm.Traverse():
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
            if prim.IsA(UsdPhysics.RevoluteJoint):
                drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
                drive.CreateTypeAttr('force')
                drive.CreateStiffnessAttr(900)
                drive.CreateDampingAttr(70)
                drive.CreateMaxForceAttr(100)
            elif prim.IsA(UsdPhysics.PrismaticJoint):
                drive = UsdPhysics.DriveAPI.Apply(prim, 'linear')
                drive.CreateStiffnessAttr(1500)
                drive.CreateDampingAttr(60)
                drive.CreateMaxForceAttr(10)
        arm.SetDefaultPrim(arm.GetPrimAtPath('/nero'))
        arm.Flatten().Export(str(ROOT / 'assets/robot/nero.usdc'))

    def pose(prim, xyz):
        attr = prim.GetAttribute('xformOp:translate')
        if attr:
            attr.Set(Gf.Vec3d(*xyz))
        else:
            UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*xyz))

    def fixed(stage, name, parent, child, xyz, rotation=(1, 0, 0, 0)):
        j = UsdPhysics.FixedJoint.Define(stage, '/Robot/' + name)
        j.CreateBody0Rel().SetTargets([parent])
        j.CreateBody1Rel().SetTargets([child])
        j.CreateLocalPos0Attr(Gf.Vec3f(*xyz))
        j.CreateLocalPos1Attr(Gf.Vec3f(0))
        j.CreateLocalRot0Attr(Gf.Quatf(*rotation))
        j.CreateLocalRot1Attr(Gf.Quatf(1))

    out = ROOT / 'assets/robot/locomanipulator.usda'
    stage = Usd.Stage.CreateNew(str(out))
    robot = UsdGeom.Xform.Define(stage, '/Robot').GetPrim()
    stage.SetDefaultPrim(robot)
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    robot.GetReferences().AddReference('amr.usda')
    robot.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    robot.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
    base = stage.GetPrimAtPath('/Robot/base_link')
    UsdPhysics.ArticulationRootAPI.Apply(base)
    articulation = PhysxSchema.PhysxArticulationAPI.Apply(base)
    articulation.CreateEnabledSelfCollisionsAttr(False)
    articulation.CreateSolverPositionIterationCountAttr(32)
    articulation.CreateSolverVelocityIterationCountAttr(4)
    riser = UsdGeom.Xform.Define(stage, '/Robot/riser_link').GetPrim()
    pose(riser, (0, 0, .245))
    visual = UsdGeom.Xform.Define(stage, '/Robot/riser_link/visuals').GetPrim()
    visual.GetReferences().AddReference('components/riser.usda', '/Component')
    UsdPhysics.RigidBodyAPI.Apply(riser)
    mass = UsdPhysics.MassAPI.Apply(riser)
    mass.CreateMassAttr(8.100080)
    mass.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0.256500))
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(0.537006, 0.559879, 0.171462))
    for prim in Usd.PrimRange(visual):
        if prim.IsA(UsdGeom.Mesh) and any(n in prim.GetName() for n in ('plate', 'column', 'rail')):
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
    fixed(stage, 'riser_joint', '/Robot/base_link', '/Robot/riser_link', (0, 0, .095))
    mount = UsdGeom.Xform.Define(stage, '/Robot/nero').GetPrim()
    mount.GetReferences().AddReference('nero.usdc')
    pose(mount, (0, 0, .795))
    UsdGeom.Xformable(mount).AddRotateZOp().Set(180)
    fixed(stage, 'nero_mount_joint', '/Robot/riser_link', '/Robot/nero/nero_base_link', (0, 0, .55), (0, 0, 0, 1))
    gripper = next(p for p in Usd.PrimRange(mount) if p.GetName() == 'nero_gripper_base')
    wrist = UsdGeom.Xform.Define(stage, str(gripper.GetPath()) + '/wrist_camera_link').GetPrim()
    pose(wrist, (.067, 0, .065))
    UsdGeom.Xformable(wrist).AddRotateYOp().Set(-90)
    UsdGeom.Xform.Define(stage, str(wrist.GetPath()) + '/visuals').GetPrim().GetReferences().AddReference('components/gemini305.usda', '/Component')
    cam = UsdGeom.Camera.Define(stage, str(wrist.GetPath()) + '/camera')
    cam.AddRotateXYZOp().Set(Gf.Vec3f(90, 0, -90))
    cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateVerticalApertureAttr(20.955/1.6)
    cam.CreateFocalLengthAttr(20.955 / (2*math.tan(math.radians(94/2))))
    cam.CreateClippingRangeAttr(Gf.Vec2f(.04, 1.0))
    camera_body = UsdGeom.Cube.Define(stage, str(wrist.GetPath()) + '/collision')
    camera_body.CreateSizeAttr(1)
    camera_body.AddTranslateOp().Set(Gf.Vec3d(-.0115, 0, 0))
    camera_body.AddScaleOp().Set(Gf.Vec3f(.023, .042, .042))
    camera_body.CreateVisibilityAttr('invisible')
    UsdPhysics.CollisionAPI.Apply(camera_body.GetPrim())
    bracket = UsdGeom.Cube.Define(stage, str(gripper.GetPath()) + '/camera_bracket')
    bracket.CreateSizeAttr(1)
    bracket.AddTranslateOp().Set(Gf.Vec3d(.044, 0, .05))
    bracket.AddScaleOp().Set(Gf.Vec3f(.008, .034, .024))
    bracket.CreateDisplayColorAttr([Gf.Vec3f(.38, .41, .44)])
    shaft = UsdGeom.Cylinder.Define(stage, str(gripper.GetPath()) + '/stylus_shaft')
    shaft.CreateRadiusAttr(.004)
    shaft.CreateHeightAttr(.095)
    shaft.AddTranslateOp().Set(Gf.Vec3d(0, 0, .173))
    shaft.CreateDisplayColorAttr([Gf.Vec3f(.25, .28, .3)])
    UsdPhysics.CollisionAPI.Apply(shaft.GetPrim())
    # A small elastomer stylus on the closed gripper makes contact unambiguous.
    tip = UsdGeom.Sphere.Define(stage, str(gripper.GetPath()) + '/button_tip')
    tip.CreateRadiusAttr(.008)
    tip.AddTranslateOp().Set(Gf.Vec3d(0, 0, .225))
    tip.CreateDisplayColorAttr([Gf.Vec3f(.08, .12, .14)])
    UsdPhysics.CollisionAPI.Apply(tip.GetPrim())
    collider = PhysxSchema.PhysxCollisionAPI.Apply(tip.GetPrim())
    collider.CreateContactOffsetAttr(.001)
    collider.CreateRestOffsetAttr(0)
    body = gripper
    while not body.HasAPI(UsdPhysics.RigidBodyAPI):
        body = body.GetParent()
    PhysxSchema.PhysxContactReportAPI.Apply(body).CreateThresholdAttr(0)
    mass = UsdPhysics.MassAPI(body)
    mass.CreateMassAttr(float(mass.GetMassAttr().Get()) + .088)
    diagonal = mass.GetDiagonalInertiaAttr().Get()
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*(float(v)+.0005 for v in diagonal)))
    stage.GetRootLayer().Save()
    from robot_description import export_descriptions
    export_descriptions(ROOT)
    print('ROBOT_BUILT', out, 'wrist', wrist.GetPath(), 'tip', tip.GetPath(), flush=True)
except BaseException:
    import traceback
    traceback.print_exc()
    exit_code = 1
finally:
    app.app.post_quit(exit_code)
    app.close()
