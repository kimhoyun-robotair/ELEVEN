#!/usr/bin/env python3
"""Create original, meter-scale furnished multi-floor environments in Blender.

Blender 4.5 LTS: blender -b -t 8 --python tools/build_worlds.py -- --world both
All meshes, surfaces, textures and furnishings are procedurally authored here.
Z is up; cabin entry faces -Y. Floor top elevations are the listed story heights.
"""
import argparse
import json
import math
import os
import random
import sys
from pathlib import Path

import bpy
import bmesh
from mathutils import Vector, Matrix

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import furniture as F


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', choices=['office', 'house', 'both'], default='both')
    parser.add_argument('--render', action='store_true')
    parser.add_argument('--render-only', action='store_true')
    parser.add_argument('--samples', type=int, default=48)
    parser.add_argument('--resolution', type=int, default=1500)
    parser.add_argument('--cameras', default='', help='Comma-separated camera names for targeted render QA')
    return parser.parse_args(sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else [])


class Context:
    def __init__(self, kind):
        self.kind = kind
        self.mesh_cache = {}
        self.mats = {}
        self.cameras = []
        self.light_objects = []
        self.textures = {}
        self.floor_groups = []
        self.furniture_groups = []
        self.elevators = []
        self.clearance_records = []
        self.root = self.group('World')
        self.building = self.group('Building', self.root)
        self.furnishing = self.group('Furniture', self.root)
        self.lifts = self.group('Elevators', self.root)
        self.lighting = self.group('Lighting', self.root)
        self.create_materials()

    def group(self, name, parent=None, loc=(0, 0, 0), semantic=None):
        ob = bpy.data.objects.new(name, None)
        bpy.context.collection.objects.link(ob)
        ob.parent = parent
        ob.location = loc
        if semantic:
            ob['testbed_path'] = semantic
        return ob

    def link(self, name, mesh, loc, parent, rotation=(0, 0, 0)):
        ob = bpy.data.objects.new(name, mesh)
        bpy.context.collection.objects.link(ob)
        ob.parent = parent
        ob.location = loc
        ob.rotation_euler = rotation
        return ob

    def box(self, name, loc, size, mat, parent=None, bevel=.018):
        size = tuple(round(float(s), 5) for s in size)
        bevel = min(bevel, min(size) * .22)
        key = ('box', size, mat.name, round(bevel, 5))
        if key not in self.mesh_cache:
            bm = bmesh.new()
            bmesh.ops.create_cube(bm, size=1)
            bmesh.ops.transform(bm, matrix=Matrix.Diagonal(Vector((*size, 1))), verts=bm.verts)
            if bevel > .0001:
                bmesh.ops.bevel(bm, geom=list(bm.edges), offset=bevel, segments=2,
                                affect='EDGES', clamp_overlap=True)
            bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
            mesh = bpy.data.meshes.new(name + '_Mesh')
            bm.to_mesh(mesh)
            bm.free()
            mesh.materials.append(mat)
            uv = mesh.uv_layers.new(name='UVMap')
            for poly in mesh.polygons:
                axes = [0, 1, 2]
                axes.remove(max(axes, key=lambda a: abs(poly.normal[a])))
                for li in poly.loop_indices:
                    co = mesh.vertices[mesh.loops[li].vertex_index].co
                    uv.data[li].uv = (co[axes[0]], co[axes[1]])
            self.mesh_cache[key] = mesh
        ob = self.link(name, self.mesh_cache[key], loc, parent)
        if any(s in name.lower() for s in ('ceiling', 'coffer', 'cove_', 'luminaire')):
            ob['testbed_ceiling'] = True
        return ob

    def cylinder(self, name, loc, radius, depth, mat, parent=None, rotation=(0, 0, 0)):
        key = ('cylinder', round(radius, 5), round(depth, 5), mat.name)
        if key not in self.mesh_cache:
            bm = bmesh.new()
            bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=24,
                                 radius1=radius, radius2=radius, depth=depth)
            mesh = bpy.data.meshes.new(name + '_Mesh')
            bm.to_mesh(mesh)
            bm.free()
            mesh.materials.append(mat)
            for poly in mesh.polygons:
                poly.use_smooth = abs(poly.normal.z) < .5
            self.mesh_cache[key] = mesh
        return self.link(name, self.mesh_cache[key], loc, parent, rotation)

    def sphere(self, name, loc, scale, mat, parent=None):
        key = ('sphere', tuple(scale), mat.name)
        if key not in self.mesh_cache:
            bm = bmesh.new()
            bmesh.ops.create_uvsphere(bm, u_segments=16, v_segments=8, radius=1)
            bmesh.ops.transform(bm, matrix=Matrix.Diagonal(Vector((*scale, 1))), verts=bm.verts)
            mesh = bpy.data.meshes.new(name + '_Mesh')
            bm.to_mesh(mesh)
            bm.free()
            mesh.materials.append(mat)
            for poly in mesh.polygons:
                poly.use_smooth = True
            self.mesh_cache[key] = mesh
        return self.link(name, self.mesh_cache[key], loc, parent)

    def text(self, name, body, loc, size, mat, parent=None, rotation=(math.pi/2,0,0), align='CENTER'):
        key=('text',body,size,mat.name,align)
        if key in self.mesh_cache:
            return self.link(name,self.mesh_cache[key],loc,parent,rotation)
        data = bpy.data.curves.new(name + '_Font', 'FONT')
        data.body = body
        data.size = size
        data.extrude = .0006
        data.bevel_depth = .0002
        data.align_x = align
        data.align_y = 'CENTER'
        ob = bpy.data.objects.new(name, data)
        bpy.context.collection.objects.link(ob)
        ob.parent = parent
        ob.location = loc
        ob.rotation_euler = rotation
        ob.data.materials.append(mat)
        # Exporters consume mesh text consistently, retaining glyph relief.
        bpy.context.view_layer.objects.active = ob
        ob.select_set(True)
        bpy.ops.object.convert(target='MESH')
        ob.select_set(False)
        self.mesh_cache[key]=ob.data
        return ob

    def area(self, name, loc, energy, size, color=(1,.89,.73), parent=None, target=None, shape='DISK'):
        data = bpy.data.lights.new(name, 'AREA')
        data.energy = energy
        data.shape = shape
        data.size = size
        data.color = color
        ob = bpy.data.objects.new(name, data)
        bpy.context.collection.objects.link(ob)
        ob.parent = parent or self.lighting
        ob.location = loc
        if target is not None:
            direction = Vector(target) - Vector(loc)
            ob.rotation_euler = direction.to_track_quat('-Z', 'Y').to_euler()
        self.light_objects.append(ob)
        return ob

    def camera(self, name, loc, target, lens=26):
        data = bpy.data.cameras.new(name)
        data.lens = lens
        data.clip_start = .04
        data.clip_end = 250
        ob = bpy.data.objects.new(name, data)
        bpy.context.collection.objects.link(ob)
        ob.location = loc
        ob.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat('-Z','Y').to_euler()
        self.cameras.append(ob)
        return ob

    def texture(self, key, base, style, roughness):
        """Deterministic authored PNG maps, deliberately portable beyond Blender."""
        if key in self.textures:
            return self.textures[key]
        folder = ROOT/'assets'/'textures'
        folder.mkdir(parents=True, exist_ok=True)
        n=256
        rng=random.Random(91271 + sum(ord(c) for c in key))
        pixels=[]
        rough=[]
        normals=[]
        for y in range(n):
            for x in range(n):
                u=x/n; v=y/n
                noise=rng.random()-.5
                if style=='wood':
                    grain=math.sin(u*280 + 2.1*math.sin(v*17) + .65*math.sin(v*59))
                    grain2=math.sin(u*920 + 4*math.sin(v*7))
                    tone=1+.105*grain+.035*grain2+.035*noise
                    nx=.065*math.cos(u*280 +2.1*math.sin(v*17)); ny=.007*noise
                elif style=='stone':
                    vein=math.sin(u*17+v*8 + 1.8*math.sin(v*12) + .65*math.sin(u*35))
                    tone=1-.065*(max(0,vein)**18)+.025*noise
                    nx=.009*noise; ny=.009*noise
                elif style=='fabric':
                    weave=.055*math.sin(x*math.pi/2)*math.sin(y*math.pi/2)
                    tone=1+weave+.075*noise
                    nx=.08*math.cos(x*math.pi/2); ny=.08*math.cos(y*math.pi/2)
                elif style=='steel':
                    tone=1+.035*noise+.025*math.sin(x*math.pi/2)
                    nx=.04*noise; ny=.001
                else:
                    tone=1+.025*noise
                    nx=.012*noise;ny=.012*noise
                pixels.extend([max(0,min(1,c*tone)) for c in base]+[1])
                r=max(.02,min(1,roughness+.055*noise))
                rough.extend([r,r,r,1])
                normals.extend([.5+nx,.5+ny,1,1])
        output=[]
        for suffix,data,colorspace in [('basecolor',pixels,'sRGB'),('roughness',rough,'Non-Color'),('normal',normals,'Non-Color')]:
            path=folder/f'{key}_{suffix}.png'
            im=bpy.data.images.new(key+'_'+suffix,width=n,height=n)
            im.colorspace_settings.name=colorspace
            im.pixels.foreach_set(data)
            im.filepath_raw=str(path)
            im.file_format='PNG'
            im.save()
            output.append(im)
        self.textures[key]=output
        return output

    def material(self, name, color, rough=.5, metal=0, texture=None, transmission=0, emission=None):
        mat=bpy.data.materials.new(name)
        mat.use_nodes=True
        mat.diffuse_color=(*color,1)
        mat.roughness=rough
        mat.metallic=metal
        bs=mat.node_tree.nodes.get('Principled BSDF')
        bs.inputs['Base Color'].default_value=(*color,1)
        bs.inputs['Roughness'].default_value=rough
        bs.inputs['Metallic'].default_value=metal
        if transmission:
            bs.inputs['Transmission Weight'].default_value=transmission
            bs.inputs['IOR'].default_value=1.46
            # Alpha works in USD Preview Surface and low cost web rendering.
            bs.inputs['Alpha'].default_value=.19
            mat.diffuse_color=(*color,.19)
            mat.surface_render_method='DITHERED'
            mat.use_transparent_shadow=True
        if emission:
            bs.inputs['Emission Color'].default_value=(*color,1)
            bs.inputs['Emission Strength'].default_value=emission
        if texture:
            images=self.texture(name,color,texture,rough)
            for ix,image in enumerate(images):
                node=mat.node_tree.nodes.new('ShaderNodeTexImage')
                node.image=image
                node.extension='REPEAT'
                if ix==0:
                    mat.node_tree.links.new(node.outputs['Color'],bs.inputs['Base Color'])
                elif ix==1:
                    mat.node_tree.links.new(node.outputs['Color'],bs.inputs['Roughness'])
                else:
                    normal=mat.node_tree.nodes.new('ShaderNodeNormalMap')
                    normal.inputs['Strength'].default_value=.22
                    mat.node_tree.links.new(node.outputs['Color'],normal.inputs['Color'])
                    mat.node_tree.links.new(normal.outputs['Normal'],bs.inputs['Normal'])
        return mat

    def create_materials(self):
        spec={
            'oak':((.54,.35,.18),.43,0,'wood'),
            'walnut':((.23,.115,.055),.36,0,'wood'),
            'white':((.79,.81,.79),.62,0,None),
            'plaster':((.78,.76,.70),.83,0,'plaster'),
            'fabric':((.24,.27,.25),.9,0,'fabric'),
            'fabric_blue':((.055,.145,.20),.88,0,'fabric'),
            'fabric_cream':((.72,.65,.53),.91,0,'fabric'),
            'black':((.017,.021,.023),.35,.25,None),
            'steel':((.49,.52,.53),.3,.92,'steel'),
            'brass':((.59,.38,.14),.29,.83,None),
            'chrome':((.7,.73,.75),.15,1,None),
            'foliage':((.09,.20,.055),.62,0,None),
            'soil':((.062,.028,.012),1,0,None),
            'ceramic':((.84,.79,.69),.2,0,None),
            'paper':((.88,.88,.82),.84,0,None),
            'rug':((.42,.39,.32),.95,0,'fabric'),
            'counter':((.81,.79,.73),.29,0,'stone'),
            'tile':((.38,.41,.40),.39,0,'stone'),
            'leather':((.32,.14,.065),.53,0,'fabric'),
            'concrete':((.41,.41,.37),.85,0,'plaster'),
            'carpet':((.20,.25,.27),.97,0,'fabric'),
        }
        for key,(color,rough,metal,tex) in spec.items():
            self.mats[key]=self.material(key,color,rough,metal,tex)
        self.mats['glass']=self.material('clear_laminated_glass',(.75,.9,.92),.065,0,transmission=.9)
        self.mats['screen']=self.material('screen',(.018,.10,.16),.25,0,emission=.45)
        self.mats['light']=self.material('architectural_3000K_light',(1,.84,.61),.2,0,emission=3)
        self.mats['cool_light']=self.material('display_light',(.39,.71,1),.23,0,emission=2)
        self.mats['off_lamp']=self.material('inactive_button',(.15,.17,.17),.26,.2)


def slab_with_holes(C,name,z,bounds,holes,parent,mat,thickness=.22):
    """Exact rectangular grid tessellation; cells inside shaft holes are omitted."""
    xmin,xmax,ymin,ymax=bounds
    xs=sorted(set([xmin,xmax]+[v for h in holes for v in h[:2]]))
    ys=sorted(set([ymin,ymax]+[v for h in holes for v in h[2:]]))
    for ix,(a,b) in enumerate(zip(xs,xs[1:])):
        for iy,(c,d) in enumerate(zip(ys,ys[1:])):
            mx=(a+b)/2;my=(c+d)/2
            if any(h[0]<mx<h[1] and h[2]<my<h[3] for h in holes):
                continue
            ob=C.box(f'{name}_{ix}_{iy}',(mx,my,z-thickness/2),(b-a,d-c,thickness),mat,parent,bevel=0)
            if len(ob.data.materials)==1:
                ob.data.materials.append(C.mats['white'])
                for polygon in ob.data.polygons:
                    if polygon.normal.z < .9:
                        polygon.material_index=1


def opening_wall(C,name,x1,x2,y,z,height,doorx,parent,mat,width=1.3,glass=False,swing=1):
    for a,b,suffix in [(x1,doorx-width/2,'left'),(doorx+width/2,x2,'right')]:
        if b-a>.01:
            C.box(name+'_'+suffix,((a+b)/2,y,z+height/2),(b-a,.14,height),mat,parent,bevel=.005)
    C.box(name+'_header',(doorx,y,z+(2.28+height)/2),(width,.14,height-2.28),mat,parent,bevel=.005)
    for dx in [-width/2,width/2]:
        C.box(name+'_doorjamb',(doorx+dx,y,z+1.14),(.04,.20,2.28),C.mats['black'],parent,.005)
    # Door is modelled parked open against the room-side wall; passage remains clear.
    leaf=C.group(name+'_OpenDoor',parent,(doorx-width/2,y+swing*.14,z))
    leaf.rotation_euler.z=swing*math.pi/2
    C.box(name+'_leaf',(width/2,0,1.11),(width-.06,.04,2.2),C.mats['glass'] if glass else C.mats['oak'],leaf,.01)
    C.box(name+'_handle',(width-.15,-.06,1.04),(.10,.025,.025),C.mats['steel'],leaf,.005)
    C.clearance_records.append({'kind':'roomDoor','name':name,'clearWidth':width-.04,'clearHeight':2.26,'center':[doorx,y,z]})


def art(C,name,parent,x,y,z,width=1.7,height=1.1):
    C.box(name+'_frame',(x,y,z),(width,.05,height),C.mats['walnut'],parent,.014)
    C.box(name+'_canvas',(x,y-.03,z),(width-.09,.009,height-.09),C.mats['paper'],parent,.001)
    for i,(cx,cz,sx,sz,m) in enumerate([(-.2,.05,.44,.58,'fabric_blue'),(.20,-.08,.35,.35,'oak'),(.33,.23,.18,.14,'brass')]):
        C.box(name+'_composition'+str(i),(x+cx*width,y-.04,z+cz*height),(sx*width,.004,sz*height),C.mats[m],parent,.012)


def exterior(C,group,w,d,z,h):
    # Sill, clerestory and regularly spaced mullions surround actual windows.
    for y in [-d/2,d/2]:
        for zz,hh in [(.39,.78),(h-.34,.68)]:
            C.box('Facade_spandrel',(0,y,z+zz),(w,.22,hh),C.mats['plaster'],group,.012)
        for x in [(-w/2)+i*2.5 for i in range(int(w/2.5)+1)]:
            C.box('Facade_mullion',(x,y,z+h/2),(.07,.26,h),C.mats['black'],group,.004)
        C.box('Facade_window',(0,y,z+(h+.10)/2),(w-.1,.025,h-1.46),C.mats['glass'],group,.003)
    for x in [-w/2,w/2]:
        for zz,hh in [(.39,.78),(h-.34,.68)]:
            C.box('Facade_spandrel',(x,0,z+zz),(.22,d,hh),C.mats['plaster'],group,.012)
        for y in [(-d/2)+i*2.5 for i in range(int(d/2.5)+1)]:
            C.box('Facade_mullion',(x,y,z+h/2),(.26,.07,h),C.mats['black'],group,.004)
        C.box('Facade_window',(x,0,z+(h+.10)/2),(.025,d-.1,h-1.46),C.mats['glass'],group,.003)
    # Baseboards maintain a 2mm intentional finish offset from plaster.
    for y in [-d/2+.13,d/2-.13]:
        C.box('Facade_baseboard',(0,y,z+.065),(w-.3,.025,.13),C.mats['oak'],group,.003)


def elevator(C,eid,style,x,y,floors):
    root=C.group(eid,C.lifts,(x,y,0),f'/World/Elevators/{eid}')
    root['testbed_elevatorId']=eid
    root['testbed_type']=style
    cabin=C.group(eid+'_Cabin',root,semantic=f'/World/Elevators/{eid}/Cabin')
    interior=C.group(eid+'_Interior',cabin)
    cabin['testbed_movable']=True
    metal=C.mats['steel'] if style=='opaque' else C.mats['brass']
    panelmat=C.mats['steel'] if style=='opaque' else C.mats['glass']
    C.box(eid+'_CabinFloor',(0,0,-.07),(2.4,2.4,.14),C.mats['counter'],interior,.014)
    for xx in [-1.11,1.11]:
        C.box(eid+'_SideWall',(xx,.03,1.35),(.085,2.29,2.70),panelmat,interior,.009)
        for yy in [-1.065,1.095]:
            C.box(eid+'_CornerPost',(xx,yy,1.35),(.072,.072,2.70),metal,interior,.008)
        C.cylinder(eid+'_SideHandrail',(xx*.955,.08,.95),.022,1.85,metal,interior,(math.pi/2,0,0))
        for yy in [-.68,.72]:
            C.box(eid+'_HandrailBracket',(xx*.985,yy,.95),(.08,.03,.03),metal,interior,.006)
    C.box(eid+'_RearWall',(0,1.15,1.35),(2.28,.09,2.70),panelmat,interior,.008)
    C.cylinder(eid+'_RearHandrail',(-.22,1.054,.95),.022,1.55,metal,interior,(0,math.pi/2,0))
    # Large mirror on the opaque back wall; silver backing exports as polished metal.
    if style=='opaque':
        C.box(eid+'_RearMirrorFrame',(-.27,1.091,1.81),(1.55,.022,1.25),C.mats['black'],interior,.01)
        C.box(eid+'_RearMirror',(-.27,1.075,1.81),(1.49,.008,1.19),C.mats['chrome'],interior,.004)
        for zz in [.18,2.55]:
            C.box(eid+'_BrushedSteelBand',(0,1.095,zz),(2.15,.012,.045),C.mats['chrome'],interior,.003)
    C.box(eid+'_Roof',(0,0,2.685),(2.4,2.4,.09),metal,interior,.014)
    C.box(eid+'_CeilingInset',(0,0,2.632),(2.17,2.14,.02),C.mats['white'],interior,.01)
    for xx in [-.67,.67]:
        for yy in [-.58,.58]:
            C.cylinder(eid+'_DownlightTrim',(xx,yy,2.609),.095,.024,metal,interior)
            C.cylinder(eid+'_DownlightLens',(xx,yy,2.593),.070,.007,C.mats['light'],interior)
            C.area(eid+'_Downlight',(xx,yy,2.577),35,.14,parent=cabin)
    for xx in [-1.0,1.0]:
        C.box(eid+'_VentilationSlot',(xx,.65,2.615),(.07,.57,.006),C.mats['black'],interior,.004)
    C.box(eid+'_FrontHeader',(0,-1.15,2.50),(2.4,.11,.36),metal,interior,.008)
    for xx in [-.966,.966]:
        C.box(eid+'_FrontJamb',(xx,-1.15,1.16),(.46,.11,2.32),metal,interior,.008)
    C.box(eid+'_CabinSill',(0,-1.165,-.006),(1.48,.10,.025),C.mats['steel'],interior,.002)
    for xx in [-.45,0,.45]:
        C.box(eid+'_CabinSillGroove',(xx,-1.165,.008),(.28,.003,.0015),C.mats['black'],interior,0)
    doors=C.group(eid+'_Cabin_Doors',cabin,semantic=f'/World/Elevators/{eid}/Cabin/Doors')
    door_nodes={}
    for side,sgn in [('Left',-1),('Right',1)]:
        node=C.group(eid+'_Cabin_Door_'+side,doors,semantic=f'/World/Elevators/{eid}/Cabin/Doors/{side}')
        C.box(eid+'_CabinDoorPanel_'+side,(sgn*.36,-1.24,1.16),(.706,.04,2.32),panelmat,node,.006)
        for xx in [sgn*.018,sgn*.7]:
            C.box(eid+'_CabinDoorEdge_'+side,(xx,-1.245,1.16),(.024,.045,2.32),metal,node,.003)
        if style=='glass':
            for zz in [.10,2.22]:
                C.box(eid+'_CabinDoorRail',(sgn*.36,-1.245,zz),(.70,.045,.07),metal,node,.003)
        door_nodes[side.lower()]=node.name
    panel=C.group(eid+'_Panel',cabin,semantic=f'/World/Elevators/{eid}/Cabin/Panel')
    C.box(eid+'_ControlPanelBacking',(.88,1.083,1.3275),(.36,.035,2.035),metal,panel,.018)
    buttons=[]
    def lamp(name,ptype,loc,label,floor=None,direction=None,parent=panel,semantic=None):
        group=C.group(name,parent,semantic=semantic)
        mat=C.material(name+'_Lamp_mat',(.11,.14,.15),.23,.15)
        C.cylinder(name+'_Bezel',loc,.045,.013,C.mats['chrome'],group,(math.pi/2,0,0))
        ob=C.cylinder(name+'_Lamp',(loc[0],loc[1]-.009,loc[2]),.035,.014,mat,group,(math.pi/2,0,0))
        ob['testbed_elevatorId']=eid
        ob['testbed_button']=ptype
        ob['testbed_lightMaterial']=mat.name
        group['testbed_elevatorId']=eid
        if floor is not None:
            ob['testbed_floor']=floor;group['testbed_floor']=floor
        if direction:
            ob['testbed_direction']=direction;group['testbed_direction']=direction
        C.text(name+'_Glyph',str(label),(loc[0],loc[1]-.018,loc[2]),.035,C.mats['paper'],group)
        # Braille dots below each floor numeral, separate from the illuminated ring.
        for d in range(3 if floor is None else (floor%3+1)):
            C.sphere(name+'_Braille',(loc[0]-.013+d*.013,loc[1]-.018,loc[2]-.061),(.0025,.0015,.0025),C.mats['chrome'],group)
        buttons.append({'node':ob.name,'groupNode':group.name,'material':mat.name,'floor':floor,'type':ptype,'direction':direction,'path':semantic})
        return group
    for f in range(len(floors)):
        lamp(f'{eid}_Floor_{f}','floor',(.88,1.054,1.18+f*.17),str(f+1),floor=f,semantic=f'/World/Elevators/{eid}/Cabin/Panel/Floor_{f}')
    for i,(key,label) in enumerate([('Open','< >'),('Close','> <'),('Alarm','!')]):
        lamp(f'{eid}_{key}',key.lower(),(.88,1.054,.72-i*.13),label,semantic=f'/World/Elevators/{eid}/Cabin/Panel/{key}')
    C.box(eid+'_DisplayBezel',(.88,1.052,2.17),(.29,.018,.21),C.mats['black'],panel,.012)
    for display_floor in range(len(floors)):
        digit=C.text(eid+f'_CabinFloorDisplay_{display_floor}',str(display_floor+1),(.88,1.036,2.19),.12,C.mats['cool_light'],panel)
        digit['testbed_displayFloor']=display_floor
        digit['testbed_elevatorId']=eid
        digit.hide_render=display_floor!=0
        digit.hide_set(display_floor!=0)
    C.text(eid+'_Capacity','8 PERSONS  630 kg',(.10,1.089,2.44),.05,C.mats['black'],interior)
    landing=C.group(eid+'_LandingDoors',root,semantic=f'/World/Elevators/{eid}/LandingDoors')
    hall=C.group(eid+'_HallButtons',root,semantic=f'/World/Elevators/{eid}/HallButtons')
    landing_nodes=[]
    for f,z in enumerate(floors):
        # Each landing includes its own door frame and visual shaft structure.
        fg=C.group(f'{eid}_Shaft_Floor_{f}',C.floor_groups[f])
        # Structural coordinates here are building coordinates, and therefore include x/y.
        wallmat=C.mats['plaster'] if style=='opaque' else C.mats['glass']
        wallheight=(floors[1]-floors[0])-.22
        for xx in [-1.44,1.44]:
            C.box(eid+'_ShaftSide',(x+xx,y+.19,z+wallheight/2),(.12,2.68,wallheight),wallmat,fg,.009)
        C.box(eid+'_ShaftRear',(x,y+1.48,z+wallheight/2),(3,.12,wallheight),wallmat,fg,.009)
        for xx in [-1.46,1.46]:
            for yy in [-1.46,1.46]:
                C.box(eid+'_ShaftPost',(x+xx,y+yy,z+1.65),(.11,.11,3.3),metal,fg,.007)
        for xx in [-1.39,1.39]:
            C.box(eid+'_GuideRail',(x+xx,y+.60,z+1.65),(.035,.07,3.3),C.mats['steel'],fg,.002)
        for xx in [-1.125,1.125]:
            C.box(eid+'_LandingWall',(x+xx,y-1.46,z+1.18),(.75,.14,2.36),C.mats['walnut'] if C.kind=='house' else C.mats['plaster'],fg,.006)
        C.box(eid+'_LandingHeader',(x,y-1.46,z+2.73),(3,.14,.74),C.mats['walnut'] if C.kind=='house' else C.mats['plaster'],fg,.006)
        for xx in [-.758,.758]:
            C.box(eid+'_LandingJamb',(x+xx,y-1.48,z+1.19),(.07,.14,2.38),metal,fg,.006)
        C.box(eid+'_LandingLintel',(x,y-1.48,z+2.395),(1.59,.14,.07),metal,fg,.006)
        C.box(eid+'_LandingThreshold',(x,y-1.36,z-.006),(1.49,.32,.025),C.mats['steel'],fg,.002)
        C.box(eid+'_HallDisplay',(x,y-1.546,z+2.68),(.43,.025,.21),C.mats['black'],fg,.009)
        for df in range(len(floors)):
            digit=C.text(eid+f'_HallFloor_{f}_Display_{df}',str(df+1),(x,y-1.565,z+2.70),.135,C.mats['cool_light'],fg)
            digit['testbed_displayFloor']=df
            digit['testbed_elevatorId']=eid
            digit.hide_render=df!=0
            digit.hide_set(df!=0)
        C.text(eid+f'_HallLabel_{f}',eid+'  /  '+('GLASS' if style=='glass' else 'PASSENGER'),(x,y-1.552,z+3.0),.075,C.mats['white'] if C.kind=='house' else C.mats['black'],fg)
        lfg=C.group(f'{eid}_Landing_Floor_{f}',landing,(0,0,z),f'/World/Elevators/{eid}/LandingDoors/Floor_{f}')
        drecord={'floor':f,'z':z}
        for side,sgn in [('Left',-1),('Right',1)]:
            dg=C.group(f'{eid}_Landing_F{f}_{side}',lfg,semantic=f'/World/Elevators/{eid}/LandingDoors/Floor_{f}/{side}')
            C.box(eid+'_LandingPanel',(sgn*.36,-1.34,1.17),(.706,.045,2.34),panelmat,dg,.006)
            for xx in [sgn*.017,sgn*.7]:
                C.box(eid+'_LandingPanelEdge',(xx,-1.369,1.17),(.022,.02,2.34),metal,dg,.003)
            if style=='glass':
                for zz in [.08,2.26]:
                    C.box(eid+'_LandingGlassRail',(sgn*.36,-1.369,zz),(.7,.026,.065),metal,dg,.003)
            drecord[side.lower()]=dg.name
        landing_nodes.append(drecord)
        hfg=C.group(f'{eid}_Hall_Floor_{f}',hall,(0,0,z),f'/World/Elevators/{eid}/HallButtons/Floor_{f}')
        C.box(eid+'_HallButtonPlate',(1.095,-1.55,1.10),(.22,.022,.37),metal,hfg,.017)
        if f<len(floors)-1:
            lamp(f'{eid}_Hall_F{f}_Up','hall',(1.095,-1.577,1.19),'^',floor=f,direction='up',parent=hfg,semantic=f'/World/Elevators/{eid}/HallButtons/Floor_{f}/Up')
        if f>0:
            lamp(f'{eid}_Hall_F{f}_Down','hall',(1.095,-1.577,1.04),'v',floor=f,direction='down',parent=hfg,semantic=f'/World/Elevators/{eid}/HallButtons/Floor_{f}/Down')
        C.clearance_records.append({'kind':'elevatorDoor','elevatorId':eid,'floor':f,'clearWidth':1.44,'clearHeight':2.32,'shaftBounds':[x-1.5,x+1.5,y-1.5,y+1.5],'center':[x,y-1.46,z]})
    item={'id':eid,'type':style,'position':[x,y,0],'floorHeights':floors,'doorTravel':.72,'entryLocal':[0,-1.2,0],
       'cabinSize':[2.4,2.4,2.7],'cabinDimensions':{'width':2.4,'depth':2.4,'height':2.7,'interiorWidth':2.135,'interiorDepth':2.29,'floorTop':0,'doorY':-1.24,'doorPanelWidth':.706},'cabinNode':cabin.name,'cabinDoorNodes':door_nodes,'landingDoorNodes':landing_nodes,'buttons':buttons,
          'sourcePaths':{'root':f'/World/Elevators/{eid}','cabin':f'/World/Elevators/{eid}/{cabin.name}',
                         'doors':{s:f'/World/Elevators/{eid}/{cabin.name}/{doors.name}/{n}' for s,n in door_nodes.items()}}}
    C.elevators.append(item)


def common_floor(C,f,z,w,d,h,holes):
    fg=C.group(f'Building_Floor_{f}',C.building)
    fg['testbed_floor']=f
    fg['testbed_floorGroup']=f
    furniture=C.group(f'Furniture_Floor_{f}',C.furnishing)
    furniture['testbed_floor']=f
    furniture['testbed_floorGroup']=f
    C.floor_groups.append(fg)
    C.furniture_groups.append(furniture)
    slab_with_holes(C,f'Slab_F{f}',z,(-w/2,w/2,-d/2,d/2),holes,fg,C.mats['counter'] if C.kind=='house' else C.mats['tile'])
    exterior(C,fg,w,d,z,h)
    # Continuous ceiling strips and fittings leave the shaft openings clear.
    for xx in range(-int(w/2)+2,int(w/2),4):
        for yy in range(-int(d/2)+2,int(d/2)-2,4):
            if any(a-.4<xx<b+.4 and c-.4<yy<dd+.4 for a,b,c,dd in holes):
                continue
            if C.kind=='office':
                C.box('Recessed_luminaire_trim',(xx,yy,z+h-.275),(1.24,.34,.035),C.mats['white'],fg,.008)
                C.box('Recessed_luminaire_lens',(xx,yy,z+h-.3),(1.15,.26,.016),C.mats['light'],fg,.005)
                C.area('Office_ceiling_light',(xx,yy,z+h-.34),110,1.2,(1,.94,.85))
            else:
                C.cylinder('Recessed_downlight_trim',(xx,yy,z+h-.343),.10,.018,C.mats['brass'],fg)
                C.cylinder('Recessed_downlight_lens',(xx,yy,z+h-.358),.072,.012,C.mats['light'],fg)
                C.area('House_ceiling_light',(xx,yy,z+h-.385),78,.24,(1,.88,.71))
    return fg,furniture


def office(C):
    floors=[0,3.6,7.2,10.8]
    positions=[(-10.5,8.5),(-3.5,8.5),(3.5,8.5),(10.5,8.5)]
    holes=[(x-1.5,x+1.5,y-1.5,y+1.5) for x,y in positions]
    for f,z in enumerate(floors):
        fg,ff=common_floor(C,f,z,30,24,3.6,holes)
        # Four enclosed office suites per level around a generous central spine.
        rooms=[(-14.8,-8),(-8,-2.6),(2.6,8),(8,14.8)]
        for i,(a,b) in enumerate(rooms):
            mx=(a+b)/2
            room=C.group(f'Office_F{f}_Room_{i+1}',ff)
            C.box('Office_carpet',(mx,-1.35,z+.011),(b-a-.16,8.76,.02),C.mats['carpet'],room,.006)
            C.box('Office_rear_partition',(mx,-5.84,z+1.58),(b-a,.14,3.16),C.mats['glass'],fg,.004)
            for yy in [-5.84,3.14]:
                C.box('Partition_top_rail',(mx,yy,z+3.11),(b-a,.045,.075),C.mats['black'],fg,.002)
            opening_wall(C,f'Office_{f}_{i}_entry',a,b,3.14,z,3.17,mx,fg,C.mats['glass'],1.3,True,swing=-1)
            for xx in ([a,b] if i in [0,2] else [b]):
                C.box('Office_side_partition',(xx,-1.35,z+1.59),(.14,9,3.18),C.mats['plaster'],fg,.008)
                C.box('Office_baseboard',(xx+.075,-1.35,z+.07),(.012,8.95,.14),C.mats['oak'],fg,.002)
            C.text('Room_number',f'{f+1}0{i+1}',(mx-1.2,3.048,z+1.65),.14,C.mats['black'],fg)
            if i==1:
                F.meeting_table(C,room,mx,-1.4,z+.021,0)
                C.box('Meeting_wall_display',(a+.10,-1.3,z+1.78),(.04,1.8,1.05),C.mats['black'],room,.03)
                C.box('Meeting_screen',(a+.074,-1.3,z+1.78),(.009,1.69,.94),C.mats['screen'],room,.008)
                F.bookshelf(C,room,mx,-5.42,z+.021,0)
            else:
                for yy in [-3.50,.25]:
                    for xx in [mx-1.28,mx+1.28]:
                        F.desk(C,room,xx,yy,z+.021,0)
                        F.office_chair(C,room,xx,yy-.80,z+.021,math.pi)
                F.bookshelf(C,room,mx,-5.4,z+.021,0)
            F.plant(C,room,b-.65,2.5,z+.021,0)
        # South work lounge: reception on ground level, shared tables on upper levels.
        if f==0:
            reception=C.group('Reception',ff)
            C.box('Reception_desk',(0,-7.7,z+.56),(4.4,1.05,1.12),C.mats['oak'],reception,.045)
            C.box('Reception_stone_counter',(0,-7.7,z+1.16),(4.55,1.14,.10),C.mats['counter'],reception,.02)
            for xx in [i*.09-2.13 for i in range(48)]:
                C.box('Reception_fluted_panel',(xx,-8.237,z+.57),(.034,.035,.92),C.mats['walnut'],reception,.006)
            C.text('Reception_sign','A T R I U M   /   W O R K S',(0,-8.26,z+.68),.155,C.mats['brass'],reception)
            for xx in [-1.35,1.35]:
                F.office_chair(C,reception,xx,-6.85,z,0)
                C.box('Reception_monitor',(xx,-7.60,z+1.44),(.48,.035,.29),C.mats['screen'],reception,.012)
        else:
            F.meeting_table(C,ff,0,-8.55,z,math.pi/2)
        for xx in [-9.8,9.8]:
            C.box('Lounge_wool_rug',(xx,-8.4,z+.018),(6.6,4.3,.027),C.mats['rug'],ff,.06)
            F.sofa(C,ff,xx,-10.1,z+.032,math.pi)
            F.armchair(C,ff,xx-2,-7.6,z+.032,math.pi/3)
            F.armchair(C,ff,xx+2,-7.6,z+.032,-math.pi/3)
            F.coffee_table(C,ff,xx,-8.45,z+.032,0)
            F.plant(C,ff,xx+2.4,-10.5,z,0)
        # Lift lobby furniture and clear 3.2 m circulation in front of all doors.
        for xx in [-7,0,7]:
            F.plant(C,ff,xx,7.95,z,0)
        C.box('Lobby_wayfinding_plinth',(13.65,5.35,z+.75),(.55,.30,1.5),C.mats['black'],ff,.025)
        C.text('Lobby_wayfinding',f'LEVEL  {f+1}\nOFFICES  /  MEETING\nLIFTS  E1 - E4',(13.65,5.185,z+1.2),.075,C.mats['paper'],ff)
        # Decorative ceiling slats over the main circulation spine.
        for xx in [i*.22-2.3 for i in range(22)]:
            C.box('Spine_ceiling_baffle',(xx,-.2,z+3.31),(.055,7.8,.14),C.mats['oak'],fg,.008)
        # An accessible double-height visual rhythm without inaccessible rooms.
        for xx in [-14.4,14.4]:
            F.plant(C,ff,xx,4.55,z,0)
    roof=C.group('Building_Roof',C.building)
    slab_with_holes(C,'Office_roof',14.4,(-15,15,-12,12),[],roof,C.mats['concrete'])
    for i,(x,y) in enumerate(positions):
        elevator(C,'E'+str(i+1),'opaque' if i%2==0 else 'glass',x,y,floors)
    C.camera('office_lobby',(13.6,4.5,1.66),(-3.5,8.2,1.42),22)
    C.camera('office_workplace',(13,-10.6,1.65),(2.8,-2.0,1.24),22)
    C.camera('office_cabin_opaque',(-10.5,7.93,1.60),(-10.52,9.45,1.34),16)
    C.camera('office_cabin_glass',(-3.5,7.89,1.60),(-3.48,9.47,1.35),16)
    return floors,holes,[30,24]


def house_stairs(C,fg,z,f,story):
    """Straight 20 riser stair within its own 2.4 x 5.7 m slab opening."""
    if f>=2:
        return
    n=20;run=5.4/n;rise=story/n
    for i in range(n):
        yy=-2.8+(i+.5)*run
        C.box(f'Stair_{f}_tread_{i}',(0,yy,z+(i+1)*rise-.035),(2.10,run+.015,.07),C.mats['oak'],fg,.013)
        C.box(f'Stair_{f}_riser_{i}',(0,yy-run/2,z+(i+.5)*rise),(2.06,.045,rise),C.mats['white'],fg,.004)
    for xx in [-1.075,1.075]:
        for i in range(0,n,2):
            yy=-2.8+(i+.5)*run
            C.cylinder('Stair_baluster',(xx,yy,z+(i+1)*rise+.43),.014,.90,C.mats['brass'],fg)
        # A real inclined rail with transform, not a staircase of separate bars.
        a=Vector((xx,-2.8,z+.97));b=Vector((xx,2.6,z+story+.90))
        rail=C.cylinder('Stair_handrail',(a+b)/2,.028,(b-a).length,C.mats['walnut'],fg)
        rail.rotation_euler=(b-a).to_track_quat('Z','Y').to_euler()
    C.box('Stair_upper_landing',(0,2.70,z+story-.07),(2.4,.22,.14),C.mats['oak'],fg,.009)
    C.box('Stair_lower_threshold',(0,-2.85,z-.035),(2.4,.12,.07),C.mats['oak'],fg,.006)
    C.clearance_records.append({'kind':'stair','floor':f,'clearWidth':2.07,'riserHeight':rise,'treadDepth':run,'run':5.4})


def house(C):
    floors=[0,3.4,6.8]
    positions=[(-4,7.4),(4,7.4)]
    shaft_holes=[(x-1.5,x+1.5,y-1.5,y+1.5) for x,y in positions]
    for f,z in enumerate(floors):
        holes=shaft_holes+([(-1.2,1.2,-2.9,2.8)] if f>0 else [])
        fg,ff=common_floor(C,f,z,24,20,3.4,holes)
        # Wide plank oak finish on living wings; stone defines public circulation.
        for side in [-1,1]:
            for i in range(27):
                xx=side*6.75-4.30+i*.325
                C.box('Oak_floor_plank',(xx,-2.6,z+.011),(.32,13.70,.018),C.mats['oak'],fg,.0015)
        house_stairs(C,fg,z,f,3.4)
        if f>0:
            for xx in [-1.23,1.23]:
                C.box('Atrium_glass_guard',(xx,-.05,z+.58),(.02,5.70,1.12),C.mats['glass'],fg,.003)
                C.box('Atrium_guard_cap',(xx,-.05,z+1.15),(.045,5.72,.045),C.mats['brass'],fg,.005)
            if f==2:
                C.box('Atrium_south_guard',(0,-2.94,z+.58),(2.5,.02,1.12),C.mats['glass'],fg,.003)
        # Ceiling coves are shallow and never intrude into the elevator shafts.
        for xx in [-6.7,6.7]:
            C.box('Ceiling_coffer',(xx,-2.5,z+3.10),(8.8,13.3,.09),C.mats['white'],fg,.012)
            for dx in [-4.2,4.2]:
                C.box('Warm_cove_diffuser',(xx+dx,-2.5,z+3.04),(.026,12.9,.018),C.mats['light'],fg,.003)
        if f==0:
            # Ground level: full kitchen, dining room, living room, study, guest bath.
            C.box('Living_room_rug',(6.8,-5.6,z+.04),(7.8,5.4,.035),C.mats['rug'],ff,.10)
            F.sofa(C,ff,6.4,-7.7,z+.058,math.pi)
            F.sofa(C,ff,9.15,-5.4,z+.058,-math.pi/2)
            F.armchair(C,ff,3.4,-4.8,z+.058,math.pi/3)
            F.coffee_table(C,ff,6.15,-5.85,z+.058,0)
            C.box('Living_fireplace_breast',(6.7,-2.77,z+1.5),(6.5,.30,3),C.mats['counter'],fg,.025)
            C.box('Living_fireplace_inset',(6.7,-2.94,z+.52),(2.3,.035,.49),C.mats['black'],ff,.012)
            C.box('Living_display',(6.7,-2.957,z+1.76),(2.6,.025,1.39),C.mats['black'],ff,.018)
            C.box('Living_screen',(6.7,-2.974,z+1.76),(2.49,.008,1.28),C.mats['screen'],ff,.005)
            F.plant(C,ff,10.7,-8.9,z,0)
            F.plant(C,ff,2.5,-8.9,z,0)
            F.kitchen(C,ff,-7.2,-6.0,z+.020,0)
            C.box('Kitchen_service_wall',(-6.7,-5.36,z+1.525),(5.45,.14,3.05),C.mats['plaster'],fg,.008)
            art(C,'Dining_art',ff,-6.7,-5.272,z+1.78,2.35,1.0)
            F.dining_set(C,ff,-6.8,.1,z+.020,0)
            F.pendant(C,ff,-6.8,.1,z+3.055,0)
            # Study and powder room have generous actual open doorways.
            opening_wall(C,'Ground_study_entry',2.1,11.8,2.9,z,3.08,6.9,fg,C.mats['plaster'],1.3)
            F.desk(C,ff,9.0,4.4,z,math.pi)
            F.office_chair(C,ff,9.0,5.25,z,0)
            F.bookshelf(C,ff,10.8,8.4,z,math.pi/2)
            F.armchair(C,ff,7.4,7.0,z,math.pi/2)
            # Ground west north corner guest bath, outside elevator shaft.
            opening_wall(C,'Ground_bath_entry',-11.8,-6.0,3.3,z,3.08,-8.8,fg,C.mats['plaster'],1.0)
            F.bathroom(C,ff,-9.4,7.1,z,0)
            C.box('Ground_bath_backing',(-9.4,7.69,z+1.5),(4.6,.14,3),C.mats['plaster'],fg,.008)
            C.box('Ground_bath_side',(-6.0,6.60,z+1.54),(.14,6.55,3.08),C.mats['plaster'],fg,.008)
            C.box('Ground_study_side',(6.0,6.42,z+1.54),(.14,6.95,3.08),C.mats['plaster'],fg,.008)
        else:
            # Four real sleeping / living suites across the upper residential levels.
            for side in [-1,1]:
                a,b=(-11.8,-2.5) if side<0 else (2.5,11.8)
                mx=(a+b)/2
                opening_wall(C,f'Suite_F{f}_{side}_entry',a,b,2.9,z,3.08,mx,fg,C.mats['plaster'],1.2,swing=-1)
                C.box('Suite_corridor_wall',(side*2.5,-3.48,z+1.54),(.14,12.76,3.08),C.mats['plaster'],fg,.008)
                if f==1:
                    F.bed(C,ff,mx,-6.6,z,0)
                    C.box('Bedroom_wool_rug',(mx,-5.85,z+.035),(5.4,5.5,.035),C.mats['rug'],ff,.06)
                    F.armchair(C,ff,mx+3,-5.6,z,-math.pi/4)
                    F.coffee_table(C,ff,mx+3,-4.4,z,0)
                    F.desk(C,ff,mx,-.15,z,math.pi)
                    F.office_chair(C,ff,mx,.65,z,0)
                    # Walk-through dressing wall with slatted wardrobe doors.
                    for k in [0,3]:
                        C.box('Wardrobe_cabinet',(mx-2.7+k*1.8,2.45,z+1.25),(1.74,.52,2.5),C.mats['walnut'],ff,.019)
                        for dx in [-.43,.43]:
                            C.box('Wardrobe_door',(mx-2.7+k*1.8+dx,2.175,z+1.25),(.84,.025,2.44),C.mats['oak'],ff,.007)
                            C.box('Wardrobe_pull',(mx-2.7+k*1.8+dx+.31,2.14,z+1.21),(.018,.025,.31),C.mats['brass'],ff,.004)
                else:
                    if side<0:
                        F.bed(C,ff,mx,-6.3,z,0)
                        F.armchair(C,ff,mx+2.9,-4.8,z,-math.pi/4)
                        F.bookshelf(C,ff,mx,.7,z,0)
                    else:
                        C.box('Family_lounge_rug',(mx,-5.0,z+.035),(7.2,6,.035),C.mats['rug'],ff,.06)
                        F.sofa(C,ff,mx,-7.35,z,math.pi)
                        F.armchair(C,ff,mx-2.8,-4.7,z,math.pi/3)
                        F.armchair(C,ff,mx+2.8,-4.7,z,-math.pi/3)
                        F.coffee_table(C,ff,mx,-5.4,z,0)
                        F.bookshelf(C,ff,mx,-2.3,z,0)
                        F.dining_set(C,ff,mx,.05,z,0)
                F.plant(C,ff,mx+3.7,-8.7,z,0)
            # Two bathroom wings accessible from the north hall.
            for side in [-1,1]:
                a,b=(-11.8,-6) if side<0 else (6,11.8)
                mx=(a+b)/2
                opening_wall(C,f'Bath_F{f}_{side}',a,b,4.0,z,3.08,mx,fg,C.mats['plaster'],1.0)
                F.bathroom(C,ff,mx,7.2,z,0)
                C.box('Bath_backing_wall',(mx,7.79,z+1.54),(5.66,.14,3.08),C.mats['plaster'],fg,.008)
                C.box('Bath_core_sidewall',(side*6.0,6.94,z+1.54),(.14,5.86,3.08),C.mats['plaster'],fg,.008)
        for xx in [-7.0,7.0]:
            F.plant(C,ff,xx,5.1,z,0)
        C.text('House_level_sign',f'LEVEL  {f+1}  /  RESIDENCE',(0,9.87,z+2.05),.20,C.mats['brass'],fg)
    roof=C.group('Building_Roof',C.building)
    slab_with_holes(C,'House_roof',10.2,(-12,12,-10,10),[],roof,C.mats['concrete'])
    for i,(x,y) in enumerate(positions):
        elevator(C,'E'+str(i+1),'opaque' if i==0 else 'glass',x,y,floors)
    C.camera('house_living',(2.5,-3.7,1.60),(7.2,-6.3,1.2),23)
    C.camera('house_kitchen',(-10.2,-8.8,1.63),(-.15,1.2,1.15),23)
    C.camera('house_elevator_hall',(9.8,2.5,1.64),(-2.3,7.0,1.38),23)
    C.camera('house_cabin_glass',(4,6.80,1.60),(4.03,8.37,1.36),16)
    return floors,shaft_holes,[24,20]


def setup_scene():
    # Object operators skip hidden glyphs, which otherwise contaminate the next world.
    for ob in list(bpy.data.objects):
        bpy.data.objects.remove(ob,do_unlink=True)
    for datablocks in [bpy.data.meshes,bpy.data.materials,bpy.data.images,bpy.data.cameras,bpy.data.lights,bpy.data.curves]:
        for block in list(datablocks):
            if block.users==0:
                datablocks.remove(block)
    scene=bpy.context.scene
    scene.unit_settings.system='METRIC'
    scene.unit_settings.scale_length=1
    scene.render.engine='CYCLES'
    scene.cycles.device='CPU'
    scene.cycles.samples=48
    scene.cycles.use_denoising=True
    scene.cycles.max_bounces=8
    scene.cycles.transparent_max_bounces=12
    scene.cycles.transmission_bounces=6
    scene.view_settings.view_transform='AgX'
    scene.render.image_settings.file_format='PNG'
    scene.render.resolution_x=1500
    scene.render.resolution_y=1000
    scene.render.resolution_percentage=100
    world=bpy.data.worlds.new('Architectural_daylight')
    world.use_nodes=True
    scene.world=world
    nodes=world.node_tree.nodes
    bg=nodes.get('Background')
    sky=nodes.new('ShaderNodeTexSky')
    sky.sky_type='NISHITA'
    sky.sun_elevation=math.radians(38)
    sky.sun_rotation=math.radians(135)
    sky.altitude=.08
    world.node_tree.links.new(sky.outputs['Color'],bg.inputs['Color'])
    bg.inputs['Strength'].default_value=.28
    return scene


def export(C,floors,holes,footprint):
    assets=ROOT/'assets';scenes=ROOT/'scenes'
    assets.mkdir(exist_ok=True);scenes.mkdir(exist_ok=True)
    bpy.context.scene.camera=C.cameras[0]
    bpy.context.scene['testbed_world']=C.kind
    bpy.context.scene['testbed_isaac_version']='5.1.0'
    bpy.context.scene['testbed_units']='meters'
    bpy.context.scene['testbed_license']='CC0-1.0 original procedural geometry and textures'
    bpy.ops.file.pack_all()
    bpy.ops.wm.save_as_mainfile(filepath=str(assets/f'{C.kind}.blend'),compress=True)
    display_objects=[ob for ob in bpy.data.objects if 'testbed_displayFloor' in ob]
    for ob in display_objects:
        ob.hide_set(False)
        ob.hide_render=False
    # Area lights remain in Blender/USD. glTF only supports punctual light types.
    print(f'EXPORT_GLB {C.kind}',flush=True)
    bpy.ops.export_scene.gltf(filepath=str(assets/f'{C.kind}.glb'),export_format='GLB',
        export_yup=True,export_apply=True,export_extras=True,export_cameras=True,export_lights=False,
        export_materials='EXPORT',export_texcoords=True,export_normals=True)
    print(f'EXPORT_USD {C.kind}',flush=True)
    bpy.ops.wm.usd_export(filepath=str(scenes/f'{C.kind}_geometry.usdc'),
        export_textures=True,relative_paths=True,generate_preview_surface=True,
        export_materials=True,export_uvmaps=True,export_normals=True,
        export_lights=True,export_cameras=True,export_custom_properties=True,
        evaluation_mode='RENDER')
    for ob in display_objects:
        ob.hide_set(ob['testbed_displayFloor']!=0)
        ob.hide_render=ob['testbed_displayFloor']!=0
    manifest={'schemaVersion':1,'world':C.kind,'units':'meters','upAxis':'Z','floorHeights':floors,
       'footprint':footprint,'shaftHoles':[list(h) for h in holes],
       'floorGroups':[g.name for g in C.floor_groups],
       'furnitureFloorGroups':[g.name for g in C.furniture_groups],
       'roofNode':'Building_Roof','elevators':C.elevators,'clearanceAudit':C.clearance_records,
       'cameraPresets':[{'name':cam.name,'position':list(cam.location),'quaternion':list(cam.rotation_euler.to_quaternion()),'lens':cam.data.lens} for cam in C.cameras],
       'source':'tools/build_worlds.py + tools/furniture.py','license':'CC0-1.0',
       'artifacts':{'blend':f'assets/{C.kind}.blend','glb':f'assets/{C.kind}.glb','geometry':f'scenes/{C.kind}_geometry.usdc'},
       'geometryStatistics':{'objects':len(bpy.data.objects),'meshes':len(bpy.data.meshes),'polygons':sum(len(o.data.polygons) for o in bpy.data.objects if o.type=='MESH')}}
    (assets/f'{C.kind}_manifest.json').write_text(json.dumps(manifest,indent=2))
    print('MANIFEST '+json.dumps({'world':C.kind,**manifest['geometryStatistics']}),flush=True)


def render_views(C,args):
    out=ROOT/'docs'/'renders'
    out.mkdir(parents=True,exist_ok=True)
    scene=bpy.context.scene
    scene.cycles.samples=args.samples
    scene.render.resolution_x=args.resolution
    scene.render.resolution_y=round(args.resolution*2/3)
    # Cabin doors are temporarily open for useful interior validation views.
    selected=set(args.cameras.split(',')) if args.cameras else None
    for camera in C.cameras:
        if selected and camera.name not in selected:
            continue
        scene.camera=camera
        scene.render.filepath=str(out/(camera.name+'.png'))
        bpy.ops.render.render(write_still=True)


def main():
    args=parse_args()
    names=['office','house'] if args.world=='both' else [args.world]
    for name in names:
        if args.render_only:
            bpy.ops.wm.open_mainfile(filepath=str(ROOT/'assets'/f'{name}.blend'))
            class Views:pass
            C=Views();C.cameras=[o for o in bpy.data.objects if o.type=='CAMERA']
            render_views(C,args)
            continue
        setup_scene()
        C=Context(name)
        floors,holes,footprint=office(C) if name=='office' else house(C)
        # Window-side fill models diffuse daylight without nonportable HDR assets.
        for z in floors:
            C.area('Daylight_south',(0,-footprint[1]/2+.4,z+2.4),650,footprint[0]*.7,(.73,.84,1),target=(0,0,z+1.2))
            C.area('Daylight_east',(footprint[0]/2-.4,-3,z+2.4),450,8,(.79,.87,1),target=(0,-3,z+1.2))
        export(C,floors,holes,footprint)
        if args.render:
            render_views(C,args)


if __name__=='__main__':
    main()
