#!/usr/bin/env python3
"""Build self-contained OpenUSD assets; run with Isaac Sim python.sh or usd-core.
Coordinates: metres, kilograms, seconds; +X front, +Y left, +Z up.
Visual geometry is photo-guided parametric reconstruction, not a scan.
"""
import argparse
import json
import math
from pathlib import Path
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

ROOT = Path(__file__).resolve().parents[1]
COLORS = {
    'shell': (.59,.63,.66), 'panel': (.52,.57,.60),
    'black': (.027,.033,.044), 'rubber': (.018,.021,.025),
    'metal': (.32,.36,.39), 'red': (.70,.018,.025),
    'lens': (.016,.037,.060), 'blue': (.018,.23,.65),
    'payload': (.47,.28,.12), 'floor': (.26,.29,.32),
    'white': (.82,.83,.80), 'yellow': (.9,.56,.025),
}

def physx_api(prim, name):
    # Raw registered schema tokens keep this builder usable with standard usd-core.
    # Isaac Sim resolves these from its PhysxSchema plugin when the stage is opened.
    prim.AddAppliedSchema(name)

def attr(prim, name, typ, value):
    return prim.CreateAttribute(name, typ, custom=False).Set(value)

def xform(stage, path, position=(0,0,0), rotation=None):
    x = UsdGeom.Xform.Define(stage, path)
    x.AddTranslateOp().Set(Gf.Vec3d(*position))
    if rotation is not None:
        x.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))
    return x

def materials(stage, root):
    UsdGeom.Scope.Define(stage, root+'/Looks')
    for name, color in COLORS.items():
        m = UsdShade.Material.Define(stage, root+'/Looks/'+name)
        s = UsdShade.Shader.Define(stage, m.GetPath().AppendChild('Shader'))
        s.CreateIdAttr('UsdPreviewSurface')
        s.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(color)
        s.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.28 if name in ('shell','metal','lens') else .62)
        s.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(.5 if name=='metal' else .05)
        m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), 'surface')

def finish(stage, geom, root, material, collision=False, hidden=False, contact=None):
    prim = geom.GetPrim()
    geom.CreateDisplayColorAttr([COLORS[material]])
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(UsdShade.Material.Get(stage,root+'/Looks/'+material))
    if collision:
        UsdPhysics.CollisionAPI.Apply(prim)
        physx_api(prim,'PhysxCollisionAPI')
        attr(prim,'physxCollision:contactOffset',Sdf.ValueTypeNames.Float,.002)
        attr(prim,'physxCollision:restOffset',Sdf.ValueTypeNames.Float,0.)
        if contact:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                UsdShade.Material.Get(stage,root+'/PhysicsMaterials/'+contact),
                materialPurpose='physics')
    if hidden:
        geom.CreateVisibilityAttr('invisible')
    return prim

def box(stage,path,size,pos,root,mat='shell',collision=False,hidden=False,rotation=None,contact=None):
    g = UsdGeom.Cube.Define(stage,path)
    g.CreateSizeAttr(1.)
    g.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if rotation is not None: g.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))
    g.AddScaleOp().Set(Gf.Vec3f(*size))
    g.CreateExtentAttr([Gf.Vec3f(-.5), Gf.Vec3f(.5)])
    finish(stage,g,root,mat,collision,hidden,contact)
    return g

def cylinder(stage,path,radius,width,pos,root,mat='metal',axis='Z',collision=False,hidden=False,rotation=None,contact=None):
    g=UsdGeom.Cylinder.Define(stage,path)
    g.CreateRadiusAttr(radius);g.CreateHeightAttr(width);g.CreateAxisAttr(axis)
    g.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if rotation is not None:g.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))
    ext=[radius]*3;ext['XYZ'.index(axis)]=width/2
    g.CreateExtentAttr([Gf.Vec3f(*[-v for v in ext]),Gf.Vec3f(*ext)])
    finish(stage,g,root,mat,collision,hidden,contact)
    return g

def octagon(length,width,chamfer):
    a,b=length/2,width/2;c=chamfer
    return [(a-c,-b),(a,-b+c),(a,b-c),(a-c,b),(-a+c,b),(-a,b-c),(-a,-b+c),(-a+c,-b)]

def ring_mesh(stage,path,rings,root,mat='shell',collision=False,hidden=False):
    # rings: (length, width, chamfer, z); CCW perimeter, closed watertight surface.
    pts=[]
    for length,width,chamfer,z in rings:pts.extend((x,y,z) for x,y in octagon(length,width,chamfer))
    faces=[list(reversed(range(8)))]
    for k in range(len(rings)-1):
        for i in range(8):faces.append([k*8+i,k*8+(i+1)%8,(k+1)*8+(i+1)%8,(k+1)*8+i])
    faces.append(list(range((len(rings)-1)*8,len(rings)*8)))
    g=UsdGeom.Mesh.Define(stage,path)
    g.CreatePointsAttr(pts);g.CreateFaceVertexCountsAttr([len(f) for f in faces]);g.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    g.CreateSubdivisionSchemeAttr('none');g.CreateDoubleSidedAttr(False)
    g.CreateExtentAttr([Gf.Vec3f(*[min(p[i] for p in pts) for i in range(3)]),Gf.Vec3f(*[max(p[i] for p in pts) for i in range(3)])])
    finish(stage,g,root,mat,collision,hidden,'body' if collision else None)
    if collision:UsdPhysics.MeshCollisionAPI.Apply(g.GetPrim()).CreateApproximationAttr('convexHull')
    return g

def inertia_box(m,size):
    x,y,z=size
    return [m*(y*y+z*z)/12,m*(x*x+z*z)/12,m*(x*x+y*y)/12]

def rigid(stage,path,pos,mass,inertia):
    x=xform(stage,path,pos)
    UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
    m=UsdPhysics.MassAPI.Apply(x.GetPrim());m.CreateMassAttr(mass)
    m.CreateCenterOfMassAttr(Gf.Vec3f(0));m.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
    m.CreatePrincipalAxesAttr(Gf.Quatf(1))
    physx_api(x.GetPrim(),'PhysxRigidBodyAPI')
    attr(x.GetPrim(),'physxRigidBody:enableCCD',Sdf.ValueTypeNames.Bool,True)
    attr(x.GetPrim(),'physxRigidBody:maxAngularVelocity',Sdf.ValueTypeNames.Float,10000.)
    attr(x.GetPrim(),'physxRigidBody:linearDamping',Sdf.ValueTypeNames.Float,0.)
    attr(x.GetPrim(),'physxRigidBody:angularDamping',Sdf.ValueTypeNames.Float,0.)
    return x

def joint(stage,path,body0,body1,axis,p0,p1=(0,0,0),drive=None):
    j=UsdPhysics.RevoluteJoint.Define(stage,path)
    j.CreateBody0Rel().SetTargets([body0]);j.CreateBody1Rel().SetTargets([body1])
    j.CreateAxisAttr(axis);j.CreateLocalPos0Attr(Gf.Vec3f(*p0));j.CreateLocalPos1Attr(Gf.Vec3f(*p1))
    j.CreateLocalRot0Attr(Gf.Quatf(1));j.CreateLocalRot1Attr(Gf.Quatf(1))
    j.CreateCollisionEnabledAttr(False)
    if drive:
        d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular')
        d.CreateTypeAttr('force');d.CreateStiffnessAttr(0.)
        d.CreateDampingAttr(drive['velocity_damping']);d.CreateMaxForceAttr(drive['torque_limit_nm'])
        d.CreateTargetVelocityAttr(0.) # USD degrees/s; runtime API uses rad/s.
    return j

def physics_materials(stage,root,cfg):
    for name,mu in [('drive',cfg['contact']['drive_friction']),('caster',cfg['contact']['caster_friction']),('body',cfg['contact']['body_friction'])]:
        m=UsdShade.Material.Define(stage,root+'/PhysicsMaterials/'+name)
        p=UsdPhysics.MaterialAPI.Apply(m.GetPrim());p.CreateStaticFrictionAttr(mu);p.CreateDynamicFrictionAttr(mu);p.CreateRestitutionAttr(0.)
        physx_api(m.GetPrim(),'PhysxMaterialAPI')
        attr(m.GetPrim(),'physxMaterial:frictionCombineMode',Sdf.ValueTypeNames.Token,'average')
        attr(m.GetPrim(),'physxMaterial:restitutionCombineMode',Sdf.ValueTypeNames.Token,'min')

def build_robot(cfg,out):
    stage=Usd.Stage.Open(Sdf.Layer.CreateNew(str(out), args={'format':'usda'}));root='/Robot'
    r=xform(stage,root);stage.SetDefaultPrim(r.GetPrim())
    UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1.)
    UsdPhysics.SetStageKilogramsPerUnit(stage,1.)
    stage.SetTimeCodesPerSecond(cfg['simulation']['physics_hz'])
    UsdPhysics.ArticulationRootAPI.Apply(r.GetPrim());physx_api(r.GetPrim(),'PhysxArticulationAPI')
    attr(r.GetPrim(),'physxArticulation:enabledSelfCollisions',Sdf.ValueTypeNames.Bool,False)
    attr(r.GetPrim(),'physxArticulation:solverPositionIterationCount',Sdf.ValueTypeNames.Int,32)
    attr(r.GetPrim(),'physxArticulation:solverVelocityIterationCount',Sdf.ValueTypeNames.Int,1)
    r.GetPrim().SetCustomData({'modelingMethod':'photo-guided parametric approximation','emptyMassKg':100.,'payloadLimitKg':250.,'maxLinearSpeedMps':2.,'sourcePhotoCount':8})
    materials(stage,root);physics_materials(stage,root,cfg)
    z=cfg['base_link_z_m'];base=root+'/base_link'
    rigid(stage,base,(0,0,z),cfg['base_mass_kg'],inertia_box(cfg['base_mass_kg'],cfg['base_inertia_box_m']))
    UsdGeom.Scope.Define(stage,base+'/Visuals');UsdGeom.Scope.Define(stage,base+'/Collisions')
    v=base+'/Visuals';c=base+'/Collisions'
    # Body and upper plate fit the supplied outer envelope; invisible colliders are independent.
    ring_mesh(stage,v+'/Body',[(.760,.544,.040,.052-z),(.788,.564,.042,.065-z),(.788,.564,.042,.215-z),(.775,.550,.040,.228-z)],root)
    ring_mesh(stage,v+'/WaistShadow',[(.779,.556,.043,.216-z),(.779,.556,.043,.234-z)],root,'black')
    ring_mesh(stage,v+'/TopDeck',[(.796,.576,.033,.229-z),(.800,.580,.034,.233-z),(.800,.580,.034,.241-z),(.792,.572,.030,.2447-z)],root,'black')
    # Keep the full caster sweep and drive wheels clear of the chassis collision volume.
    ring_mesh(stage,c+'/Chassis',[(.788,.564,.042,.205-z),(.788,.564,.042,.220-z)],root,'shell',True,True)
    box(stage,c+'/Spine',(.760,.300,.153),(0,0,.1285-z),root,'shell',True,True,contact='body')
    for name,sign in [('Front',1),('Rear',-1)]:
        box(stage,c+'/'+name+'Bumper',(.020,.480,.140),(sign*.384,0,.135-z),root,'shell',True,True,contact='body')
    for name,sign in [('Left',1),('Right',-1)]:
        box(stage,c+'/'+name+'Rail',(.710,.012,.095),(0,sign*.276,.1575-z),root,'shell',True,True,contact='body')
    ring_mesh(stage,c+'/Deck',[(.800,.580,.034,.229-z),(.800,.580,.034,.245-z)],root,'black',True,True)
    # Long side inset panels, service ports, status window, sockets and fasteners.
    for side,s in [('left',1),('right',-1)]:
        box(stage,v+'/'+side+'_panel',(.672,.003,.134),(0,s*.283,.136-z),root,'panel')
        box(stage,v+'/'+side+'_panel_inset',(.651,.002,.120),(0,s*.285,.136-z),root,'shell')
        for px in [-.27,.27]:
            for pz in [.090,.116]:
                cylinder(stage,f'{v}/{side}_vent_{px}_{pz}'.replace('-','n').replace('.','_'),.007,.003,(px,s*.287,pz-z),root,'black',axis='Y')
        if s==-1:
            box(stage,v+'/StatusBezel',(.126,.004,.048),(.13,-.287,.171-z),root,'metal')
            box(stage,v+'/StatusWindow',(.112,.001,.033),(.13,-.2895,.171-z),root,'black')
            for i in range(3):
                cylinder(stage,v+f'/Control_{i}',.007,.004,(-.11+i*.025,-.287,.164-z),root,'metal',axis='Y')
            box(stage,v+'/BlueSwitch',(.020,.004,.009),(-.10,-.287,.123-z),root,'blue')
        else:
            box(stage,v+'/ServicePanel',(.160,.004,.102),(.16,.287,.137-z),root,'panel')
            for i in range(4):
                cylinder(stage,v+f'/ServicePort_{i}',.013,.004,(.12+(i%2)*.067,.287,.112+(i//2)*.048-z),root,'black',axis='Y')
    # Front/rear panel details and D455-shaped 124 x 29 x 26 mm housings.
    for name,s in [('front',1),('rear',-1)]:
        box(stage,v+'/'+name+'_face',(.002,.468,.130),(s*.394,0,.138-z),root,'shell')
        # Black camera backing is flush with the front face. Camera looks outward from x=+/-.4.
        box(stage,v+'/'+name+'_camera_recess',(.001,.137,.041),(s*.3955,0,.172-z),root,'black')
        px,py,pz,yaw=cfg['camera'][name+'_pose_m_deg']
        link=xform(stage,base+'/'+name+'_camera_link',(px,py,pz-z),(0,0,yaw))
        lp=str(link.GetPath())
        box(stage,lp+'/Housing',(.026,.124,.029),(-.016,0,0),root,'metal')
        box(stage,lp+'/Face',(.001,.120,.026),(-.0025,0,0),root,'black')
        for i,py in enumerate([-.0475,0,.0475]):
            cylinder(stage,lp+f'/Lens_{i}',.008 if i!=1 else .005,.001,(-.001,py,0),root,'lens',axis='X')
        cam=UsdGeom.Camera.Define(stage,lp+'/camera')
        cam.AddRotateXYZOp().Set(Gf.Vec3f(90,0,-90))
        aperture=20.955;aspect=cfg['camera']['width']/cfg['camera']['height']
        cam.CreateHorizontalApertureAttr(aperture);cam.CreateVerticalApertureAttr(aperture/aspect)
        cam.CreateFocalLengthAttr(aperture/(2*math.tan(math.radians(cfg['camera']['hfov_deg']/2))))
        cam.CreateClippingRangeAttr(Gf.Vec2f(.01,100.))
        cam.CreateFocusDistanceAttr(1.)
        for py in [-.196,.196]:
            for pz in [.089,.113]:
                cylinder(stage,f'{v}/{name}_vent_{py}_{pz}'.replace('-','n').replace('.','_'),.0065,.002,(s*.396,py,pz-z),root,'black',axis='X')
        box(stage,v+'/'+name+'_LowerPort',(.001,.075,.020),(s*.396,0,.105-z),root,'metal')
        box(stage,v+'/'+name+'_LowerPortInset',(.001,.059,.009),(s*.397,0,.105-z),root,'black')
    # Two diagonal scanner housings below the deck and corner E-stop details.
    for name,key in [('front_right','front_right_pose_m_deg'),('rear_left','rear_left_pose_m_deg')]:
        px,py,pz,yaw=cfg['lidar'][key]
        lp=base+'/'+name+'_lidar_link';xform(stage,lp,(px,py,pz-z),(0,0,yaw))
        cylinder(stage,lp+'/Housing',.018,.032,(-.020,0,-.004),root,'black')
        cylinder(stage,lp+'/Cap',.019,.004,(-.020,0,.014),root,'metal')
        # E-stop buttons on the diagonal face, within deck footprint.
        bx,by=(.372,-.260) if name=='front_right' else (-.372,.260)
        ep=v+'/'+name+'_estop';xform(stage,ep,(bx,by,.137-z),(0,90,yaw))
        cylinder(stage,ep+'/Surround',.022,.004,(0,0,0),root,'metal')
        cylinder(stage,ep+'/RedButton',.017,.010,(0,0,.006),root,'red')
        cylinder(stage,ep+'/Center',.012,.001,(0,0,.012),root,'red')
    # Two top mounting rails with crossed ribs and recessed fastener details.
    for j,py in enumerate([-.168,.168]):
        box(stage,v+f'/TopRail_{j}',(.514,.068,.0008),(0,py,.2444-z),root,'metal')
        box(stage,v+f'/TopRailInset_{j}',(.506,.060,.0001),(0,py,.24485-z),root,'black')
        for k in range(5):
            px=-.20+k*.10
            for direction in [-1,1]:
                box(stage,v+f'/RailRib_{j}_{k}_{direction}'.replace('-','n'),(.105,.0018,.0002),(px,py,.2449-z),root,'metal',rotation=(0,0,direction*29))
        for k,px in enumerate([-.24,.24]):
            cylinder(stage,v+f'/RailBolt_{j}_{k}',.004,.0005,(px,py,.2447-z),root,'metal')
    for i,(px,py) in enumerate([(-.32,-.22),(.32,-.22),(-.32,.22),(.32,.22),(-.32,0),(.32,0)]):
        cylinder(stage,v+f'/DeckBolt_{i}',.0035,.0005,(px,py,.244-z),root,'metal')
    # Six road-contact wheels: two driven cylinders; each caster has a real swivel and roll DOF.
    d=cfg['drive'];rad=d['radius_m'];w=d['width_m'];m=d['wheel_mass_kg']
    transverse=m*(3*rad*rad+w*w)/12;axial=m*rad*rad/2
    for name,sgn in [('left',1),('right',-1)]:
        wp=root+'/'+name+'_drive';rigid(stage,wp,(0,sgn*d['track_m']/2,rad),m,[transverse,axial,transverse])
        cylinder(stage,wp+'/Tire',rad,w,(0,0,0),root,'rubber',axis='Y')
        cylinder(stage,wp+'/Collision',rad,w,(0,0,0),root,'rubber',axis='Y',collision=True,hidden=True,contact='drive')
        cylinder(stage,wp+'/Hub',rad*.58,w+.002,(0,0,0),root,'metal',axis='Y')
        joint(stage,root+'/Joints/'+name+'_drive_joint',base,wp,'Y',(0,sgn*d['track_m']/2,rad-z),drive=d)
    c=cfg['caster'];rad=c['radius_m'];w=c['width_m'];m=c['wheel_mass_kg']
    for key,sx,sy in [('fl',1,1),('fr',1,-1),('rl',-1,1),('rr',-1,-1)]:
        pivot=(sx*c['pivot_x_m'],sy*c['pivot_y_m'],c['pivot_z_m'])
        sp=root+'/caster_'+key+'_swivel';wp=root+'/caster_'+key+'_wheel'
        rigid(stage,sp,pivot,c['swivel_mass_kg'],inertia_box(c['swivel_mass_kg'],(.060,.052,.070)))
        cylinder(stage,sp+'/Bearing',.024,.010,(0,0,0),root,'metal',collision=True,contact='body')
        for s in [-1,1]:box(stage,sp+('/ForkL' if s==1 else '/ForkR'),(.034,.006,.059),(-c['trail_m']/2,s*(w/2+.005),-.03),root,'metal',collision=True,contact='body')
        wheelpos=(pivot[0]-c['trail_m'],pivot[1],rad)
        transverse=m*(3*rad*rad+w*w)/12;axial=m*rad*rad/2
        rigid(stage,wp,wheelpos,m,[transverse,axial,transverse])
        cylinder(stage,wp+'/Tire',rad,w,(0,0,0),root,'rubber',axis='Y')
        cylinder(stage,wp+'/Collision',rad,w,(0,0,0),root,'rubber',axis='Y',collision=True,hidden=True,contact='caster')
        cylinder(stage,wp+'/Hub',rad*.55,w+.002,(0,0,0),root,'metal',axis='Y')
        joint(stage,root+'/Joints/caster_'+key+'_swivel_joint',base,sp,'Z',(pivot[0],pivot[1],pivot[2]-z))
        joint(stage,root+'/Joints/caster_'+key+'_roll_joint',sp,wp,'Y',(-c['trail_m'],0,rad-pivot[2]))
    stage.GetRootLayer().Save()
    return stage

def apply_payload(stage, kg, cfg, robot_path='/World/Robot'):
    """Combine a centred fixed payload into chassis mass/COM/inertia; no extra DOF.
    Must be called before starting physics. Does not edit the referenced source USD.
    """
    if not math.isfinite(kg) or not 0<=kg<=cfg['max_payload_kg']:raise ValueError('payload must be 0..250 kg')
    base=robot_path+'/base_link';m0=cfg['base_mass_kg'];size=cfg['payload']['size_m']
    d=cfg['dimensions']['height']+size[2]/2-cfg['base_link_z_m'];com=kg*d/(m0+kg)
    i0=inertia_box(m0,cfg['base_inertia_box_m']);ip=inertia_box(kg,size)
    moment=m0*com*com+kg*(d-com)**2
    diag=[i0[0]+ip[0]+moment,i0[1]+ip[1]+moment,i0[2]+ip[2]]
    m=UsdPhysics.MassAPI(stage.GetPrimAtPath(base));m.CreateMassAttr(m0+kg)
    m.CreateCenterOfMassAttr(Gf.Vec3f(0,0,com));m.CreateDiagonalInertiaAttr(Gf.Vec3f(*diag))
    path=base+'/Payload'
    if kg>0:
        box(stage,path,size,(0,0,d),robot_path,'payload',collision=True,contact='body')
        stage.GetPrimAtPath(path).SetActive(True)
    elif stage.GetPrimAtPath(path):stage.GetPrimAtPath(path).SetActive(False)
    return {'payload_kg':kg,'total_mass_kg':cfg['mass_kg']+kg,'base_com_z_m':com,'base_inertia_kgm2':diag}

def build_world(cfg,out,payload=0.):
    stage=Usd.Stage.Open(Sdf.Layer.CreateNew(str(out), args={'format':'usda'}));root='/World';r=xform(stage,root);stage.SetDefaultPrim(r.GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z');UsdPhysics.SetStageKilogramsPerUnit(stage,1.)
    stage.SetTimeCodesPerSecond(cfg['simulation']['physics_hz'])
    materials(stage,root);physics_materials(stage,root,cfg)
    scene=UsdPhysics.Scene.Define(stage,root+'/PhysicsScene');scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));scene.CreateGravityMagnitudeAttr(9.81)
    physx_api(scene.GetPrim(),'PhysxSceneAPI')
    for name,typ,val in [('solverType',Sdf.ValueTypeNames.Token,'TGS'),('enableCCD',Sdf.ValueTypeNames.Bool,True),('enableGPUDynamics',Sdf.ValueTypeNames.Bool,False),('broadphaseType',Sdf.ValueTypeNames.Token,'MBP'),('timeStepsPerSecond',Sdf.ValueTypeNames.UInt,cfg['simulation']['physics_hz'])]:attr(scene.GetPrim(),'physxScene:'+name,typ,val)
    xform(stage,root+'/Ground')
    box(stage,root+'/Ground/Visual',(12,10,.10),(0,0,-.05),root,'floor')
    # This 5.1 test scene loses wheel support with cylinder/large-box contacts.
    ground=UsdGeom.Plane.Define(stage,root+'/Ground/CollisionPlane')
    ground.CreateAxisAttr('Z')
    finish(stage,ground,root,'floor',collision=True,hidden=True,contact='body')
    for i,(size,pos) in enumerate([((.12,10,1.4),(-6,0,.7)),((.12,10,1.4),(6,0,.7)),((12,.12,1.4),(0,-5,.7)),((12,.12,1.4),(0,5,.7))]):box(stage,root+f'/Walls/Wall_{i}',size,pos,root,'white',collision=True,contact='body')
    # Distinct targets on both sides of origin, within both RGBD sensors' valid range.
    for i,(size,pos,mat) in enumerate([((.45,.45,.65),(2.5,.45,.325),'blue'),((.55,.8,.4),(-2.5,-.35,.2),'yellow'),((.7,.35,.55),(1.4,-1.7,.275),'red'),((.4,.4,.8),(-1.5,1.5,.4),'metal')]):box(stage,root+f'/Obstacles/Box_{i}',size,pos,root,mat,collision=True,contact='body')
    for i,(px,py) in enumerate([(3,2),(-3,-2)]):cylinder(stage,root+f'/Obstacles/Post_{i}',.15,.9,(px,py,.45),root,'yellow',collision=True,contact='body')
    # Floor markings have no collision and sit below the road-contact plane to avoid z-fighting.
    for i in range(-5,6):box(stage,root+f'/Grid/X_{i}'.replace('-','n'),(.008,9.5,.0002),(i,0,.0001),root,'metal')
    for i in range(-4,5):box(stage,root+f'/Grid/Y_{i}'.replace('-','n'),(11.5,.008,.0002),(0,i,.0001),root,'metal')
    robot=stage.DefinePrim(root+'/Robot','Xform');robot.GetReferences().AddReference('./robot.usd')
    if payload:apply_payload(stage,payload,cfg)
    dome=UsdLux.DomeLight.Define(stage,root+'/DomeLight');dome.CreateIntensityAttr(900.);dome.CreateColorAttr((.82,.88,1.))
    sun=UsdLux.DistantLight.Define(stage,root+'/Sun');sun.CreateIntensityAttr(2200.);sun.CreateAngleAttr(1.)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-30,-45,20))
    camera=UsdGeom.Camera.Define(stage,root+'/OverviewCamera')
    # Author camera-to-world inverse view matrix, USD -Z forward.
    view=Gf.Matrix4d(1).SetLookAt(Gf.Vec3d(2.1,-2.4,1.7),Gf.Vec3d(0,0,.15),Gf.Vec3d(0,0,1))
    camera.AddTransformOp().Set(view.GetInverse());camera.CreateClippingRangeAttr((.01,100.));camera.CreateFocalLengthAttr(30.)
    stage.GetRootLayer().Save();return stage

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,default=ROOT/'config/robot.json');p.add_argument('--output-dir',type=Path,default=ROOT/'assets');args=p.parse_args()
    cfg=json.loads(args.config.read_text())
    if cfg['dimensions'] != {'length':0.8,'width':0.58,'height':0.245}:
        raise ValueError('This photo-guided body template is 800x580x245 mm; edit body geometry for a different envelope.')
    args.output_dir.mkdir(parents=True,exist_ok=True)
    for name in ['robot.usd','test_world.usd','test_world_payload250.usd']:
        path=args.output_dir/name
        if path.exists():path.unlink()
    robot=build_robot(cfg,args.output_dir/'robot.usd');robot.GetRootLayer().Export(str(args.output_dir/'robot.usda'))
    build_world(cfg,args.output_dir/'test_world.usd');build_world(cfg,args.output_dir/'test_world_payload250.usd',250.)
    print('Created robot.usd, robot.usda, test_world.usd, test_world_payload250.usd')

if __name__=='__main__':main()
