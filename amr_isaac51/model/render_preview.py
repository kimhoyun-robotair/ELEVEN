#!/usr/bin/env python3
"""Render a deterministic CAD-style preview directly from the generated USD.

Requires pxr (Isaac Sim Python or usd-core), numpy and matplotlib. This is a
static geometry inspection image, not a physically rendered Isaac Sim image.
No geometry is invented or modified. Inherited invisible geometry is excluded.
"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from pxr import Gf, Usd, UsdGeom

ROOT = Path(__file__).resolve().parents[1]
BG = '#f5f7f9'
INK = '#162c40'
MUTED = '#64768a'
BLUE = '#1579a8'
ORANGE = '#bf652d'


def geometry(stage):
    """Extract world-space polygons, preserving USD shape sizes and transforms."""
    cache = UsdGeom.XformCache()
    faces, colors = [], []
    for prim in stage.Traverse():
        geom = UsdGeom.Gprim(prim)
        if not geom or geom.ComputeVisibility() == UsdGeom.Tokens.invisible:
            continue
        typ = prim.GetTypeName()
        if typ not in ('Cube', 'Cylinder', 'Mesh'):
            continue
        mat = cache.GetLocalToWorldTransform(prim)
        color = geom.GetDisplayColorAttr().Get()
        color = np.array(color[0] if color else [.5, .5, .5])
        if typ == 'Cube':
            half = UsdGeom.Cube(prim).GetSizeAttr().Get() / 2
            points = [(x*half, y*half, z*half) for x, y, z in
                      [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),
                       (-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]]
            indices = [[3,2,1,0],[4,5,6,7],[0,1,5,4],
                       [1,2,6,5],[2,3,7,6],[3,0,4,7]]
        elif typ == 'Cylinder':
            cyl = UsdGeom.Cylinder(prim)
            radius, height = cyl.GetRadiusAttr().Get(), cyl.GetHeightAttr().Get()
            axis = str(cyl.GetAxisAttr().Get())
            n = 40
            points = []
            for h in (-height/2, height/2):
                for a in np.linspace(0, 2*np.pi, n, endpoint=False):
                    u, v = radius*np.cos(a), radius*np.sin(a)
                    # Cyclic coordinate permutations preserve outward winding.
                    points.append({'X': (h,u,v), 'Y': (v,h,u), 'Z': (u,v,h)}[axis])
            indices = [list(reversed(range(n))), list(range(n,2*n))]
            indices += [[i,(i+1)%n,(i+1)%n+n,i+n] for i in range(n)]
        else:
            mesh = UsdGeom.Mesh(prim)
            points = mesh.GetPointsAttr().Get()
            counts, ids = mesh.GetFaceVertexCountsAttr().Get(), mesh.GetFaceVertexIndicesAttr().Get()
            indices, start = [], 0
            for count in counts:
                indices.append(list(ids[start:start+count]))
                start += count
        points = np.array([tuple(mat.Transform(Gf.Vec3d(*p))) for p in points])
        for ids in indices:
            faces.append(points[ids])
            colors.append(color)
    return faces, colors


def basis(view, up=(0,0,1)):
    look = np.array(view, dtype=float)
    look /= np.linalg.norm(look)
    right = np.cross(up, look)
    right /= np.linalg.norm(right)
    vertical = np.cross(look, right)
    return np.array([right, vertical, look])


def draw_model(ax, faces, colors, view, up=(0,0,1), limits=None):
    proj = basis(view, up)
    ax.set_aspect('equal')
    ax.set_facecolor(BG)
    ax.set_axis_off()
    if limits:
        ax.set_xlim(*limits[0]); ax.set_ylim(*limits[1])
    else:
        pts = np.concatenate(faces) @ proj[:2].T; lo,hi=pts.min(0),pts.max(0)
        ax.set_xlim(lo[0]-.10,hi[0]+.10); ax.set_ylim(lo[1]-.10,hi[1]+.10)
    # True per-pixel depth avoids the occlusion errors of sorting entire faces.
    # Orthographic projection makes barycentric depth interpolation exact.
    xmin,xmax=ax.get_xlim(); ymin,ymax=ax.get_ylim()
    width=1600; height=max(1,int(width*(ymax-ymin)/(xmax-xmin)))
    canvas=np.ones((height,width,4)); canvas[:,:,:3]=matplotlib.colors.to_rgb(BG)
    depth=np.full((height,width),-np.inf)
    light=np.array([.6,-.8,1.5]); light/=np.linalg.norm(light)
    for poly,color in zip(faces,colors):
        normal=np.cross(poly[1]-poly[0],poly[2]-poly[0]); norm=np.linalg.norm(normal)
        if norm<1e-12: continue
        normal/=norm
        if np.dot(normal,proj[2]) < -1e-7: continue
        projected=poly @ proj.T
        projected[:,0]=(projected[:,0]-xmin)/(xmax-xmin)*width-.5
        projected[:,1]=(projected[:,1]-ymin)/(ymax-ymin)*height-.5
        shaded=np.clip(color*(.65+.33*max(0,np.dot(normal,light)))+.04,0,1)
        for k in range(1,len(poly)-1):
            tri=projected[[0,k,k+1]]
            low=np.maximum(np.floor(tri[:,:2].min(0)).astype(int),0)
            high=np.minimum(np.ceil(tri[:,:2].max(0)).astype(int),[width-1,height-1])
            if np.any(high<low): continue
            x,y=np.meshgrid(np.arange(low[0],high[0]+1),np.arange(low[1],high[1]+1))
            a,b,c=tri
            denom=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
            if abs(denom)<1e-12: continue
            u=((b[1]-c[1])*(x-c[0])+(c[0]-b[0])*(y-c[1]))/denom
            v=((c[1]-a[1])*(x-c[0])+(a[0]-c[0])*(y-c[1]))/denom
            w=1-u-v
            z=u*a[2]+v*b[2]+w*c[2]
            region=depth[low[1]:high[1]+1,low[0]:high[0]+1]
            mask=(u>=-1e-8)&(v>=-1e-8)&(w>=-1e-8)&(z>=region-1e-10)
            region[mask]=z[mask]
            canvas[low[1]:high[1]+1,low[0]:high[0]+1,:3][mask]=shaded
    ax.imshow(canvas,origin='lower',extent=(xmin,xmax,ymin,ymax),interpolation='antialiased')
    return lambda p: np.asarray(p) @ proj[:2].T


def label(ax, project, point, text, at, color=BLUE, align='left'):
    q = project(point)
    ax.plot(*q, marker='o', markersize=3.4, color=color, markeredgecolor=BG,
            markeredgewidth=.7, zorder=6)
    ax.annotate(text, xy=q, xytext=at, textcoords='data', color=color,
                fontsize=8.3, ha=align, va='center', linespacing=1.45,
                arrowprops=dict(arrowstyle='-',color=color,lw=.75,
                                connectionstyle='angle,angleA=0,angleB=90,rad=4'),
                zorder=7, bbox=dict(facecolor=BG,edgecolor='none',pad=2))


def dim(ax, start, end, text, offset=(0,0)):
    ax.annotate('', xy=end, xytext=start,
                arrowprops=dict(arrowstyle='<->', color=MUTED,lw=.8,shrinkA=0,shrinkB=0))
    center=(np.array(start)+end)/2+offset
    ax.text(*center, text, color=MUTED, fontsize=9.5, ha='center', va='center',
            rotation=90 if abs(end[0]-start[0])<1e-5 else 0,
            bbox=dict(facecolor=BG, edgecolor='none', pad=2))


def world_position(stage, path):
    return tuple(UsdGeom.XformCache().GetLocalToWorldTransform(
        stage.GetPrimAtPath(path)).ExtractTranslation())


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--usd',type=Path,default=ROOT/'assets/robot.usd')
    ap.add_argument('--output',type=Path,default=ROOT/'docs/model_preview.png')
    args=ap.parse_args()
    stage=Usd.Stage.Open(str(args.usd))
    if stage is None: raise RuntimeError(f'Cannot open {args.usd}')
    faces,colors=geometry(stage)
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10})
    fig=plt.figure(figsize=(15,10.7),facecolor=BG)
    fig.text(.042,.953,'AMR / GEOMETRY & SENSOR LAYOUT',fontsize=21,
             weight='bold',color=INK)
    fig.text(.043,.919,'800 × 580 × 245 mm   ·   100 kg empty   ·   250 kg rated payload   ·   2 m/s limit',
             fontsize=11,color=MUTED)
    panels=[(.035,.513,.45,.35),(.52,.513,.45,.35),
            (.035,.108,.45,.32),(.52,.108,.45,.32)]
    titles=['01  FRONT / RIGHT ISOMETRIC','02  TOP & SENSOR POSES',
            '03  LEFT ELEVATION','04  UNDERSIDE / SIX WHEELS']
    axes=[]
    for (x,y,w,h),title in zip(panels,titles):
        fig.text(x+.008,y+h+.019,title,fontsize=10.5,weight='bold',color=INK)
        axes.append(fig.add_axes([x,y,w,h]))
    ax=axes[0]
    p=draw_model(ax,faces,colors,(1.5,-1.7,1.1),limits=((-0.58,.58),(-.27,.50)))
    label(ax,p,world_position(stage,'/Robot/base_link/front_camera_link'),
          'Front RGB-D\nD455 housing',(.42,-.23),align='right')
    ax=axes[1]
    p=draw_model(ax,faces,colors,(0,0,1),up=(1,0,0),limits=((-0.57,.57),(-.50,.51)))
    # In this plan +X is upward, +Y (left of vehicle) is leftward on the page.
    dim(ax,(.335,-.4),(.335,.4),'800 mm',offset=(.025,0))
    dim(ax,(-.29,-.445),(.29,-.445),'580 mm')
    ax.annotate('',xy=(0,.475),xytext=(0,.418),
                arrowprops=dict(arrowstyle='-|>',color=INK,lw=.9))
    ax.text(0,.497,'FRONT +X',ha='center',va='center',color=INK,fontsize=8)
    label(ax,p,world_position(stage,'/Robot/base_link/front_camera_link'),
          'Front D455\n(+400, 0, 172)',(-.55,.39))
    label(ax,p,world_position(stage,'/Robot/base_link/rear_camera_link'),
          'Rear D455\n(−400, 0, 172)',(.36,-.35))
    label(ax,p,world_position(stage,'/Robot/base_link/front_right_lidar_link'),
          'Front-right LiDAR\n(+377, −267, 192)\nyaw −45°',(.36,.26),color=ORANGE)
    label(ax,p,world_position(stage,'/Robot/base_link/rear_left_lidar_link'),
          'Rear-left LiDAR\n(−377, +267, 192)\nyaw 135°',(-.55,-.26),color=ORANGE)
    ax=axes[2]
    p=draw_model(ax,faces,colors,(0,1,0),limits=((-.49,.51),(-.04,.37)))
    # +X is left in the left side elevation.
    dim(ax,(.445,0),(.445,.245),'245 mm',offset=(.023,0))
    ax.plot([-.405,.405],[0,0],color='#c6d0d9',lw=.65)
    ax.annotate('FRONT',xy=(-.42,.305),xytext=(-.20,.305),color=INK,fontsize=8,
                ha='center',va='center',arrowprops=dict(arrowstyle='-|>',color=INK,lw=.9))
    ax.text(0,-.033,'Visual geometry; wheel contact plane z = 0',ha='center',fontsize=8,color=MUTED)
    ax=axes[3]
    p=draw_model(ax,faces,colors,(0,0,-1),up=(1,0,0),
                 limits=((-0.57,.57),(-.49,.49)))
    label(ax,p,world_position(stage,'/Robot/right_drive'),'Driven wheel ×2\nØ200 mm (assumed)',(-.56,.13))
    label(ax,p,world_position(stage,'/Robot/caster_fl_wheel'),'Passive caster ×4\nØ80 mm (assumed)',(.34,.34),color=ORANGE)
    ax.text(.35,-.28,'Each caster:\nfree swivel + roll',ha='left',fontsize=8.4,color=MUTED,linespacing=1.5)
    fig.add_artist(plt.Line2D([.041,.959],[.079,.079],transform=fig.transFigure,
                              color='#d7dfe6',lw=.8))
    fig.text(.043,.049,'Photo-guided parametric approximation · Extracted from robot.usd · Not an Isaac Sim render',
             color=MUTED,fontsize=9)
    fig.text(.958,.049,'Sensor coordinates: (x, y, z) mm from ground-centred origin',
             color=MUTED,fontsize=8.5,ha='right')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(args.output,dpi=185,facecolor=BG)
    plt.close(fig)
    print(f'{args.output}: {len(faces)} USD faces extracted')


if __name__=='__main__':
    main()
