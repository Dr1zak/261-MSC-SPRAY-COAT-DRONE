#!/usr/bin/env python3
"""Render stills from the recorded Isaac route; no Isaac viewport is captured."""
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

from construction_site import OBSTACLES, WAYPOINTS

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'reports/x650_route_frames'
OUT.mkdir(parents=True, exist_ok=True)
rows = list(csv.DictReader((ROOT / 'reports/thrust_x650_study_route_telemetry.csv').open()))
samples = np.array([[float(row[key]) for key in ('time_s', 'x_m', 'y_m', 'z_m')]
                    for row in rows])


def box(ax, x, y, z, sx, sy, sz, color, alpha=1):
    x0, x1 = x-sx/2, x+sx/2
    y0, y1 = y-sy/2, y+sy/2
    z0, z1 = z-sz/2, z+sz/2
    faces = [
        [(x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0)],
        [(x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)],
        [(x0,y0,z0),(x1,y0,z0),(x1,y0,z1),(x0,y0,z1)],
        [(x1,y0,z0),(x1,y1,z0),(x1,y1,z1),(x1,y0,z1)],
        [(x1,y1,z0),(x0,y1,z0),(x0,y1,z1),(x1,y1,z1)],
        [(x0,y1,z0),(x0,y0,z0),(x0,y0,z1),(x0,y1,z1)],
    ]
    ax.add_collection3d(Poly3DCollection(faces, facecolors=color,
                                          edgecolors='#2c3940', linewidths=.35,
                                          alpha=alpha))


def site_3d(ax):
    for grid in np.arange(0,51,5):
        ax.plot([grid,grid],[0,50],[-.15,-.15],color='#d7c9af',lw=.65,alpha=.6)
        ax.plot([0,50],[grid,grid],[-.15,-.15],color='#d7c9af',lw=.65,alpha=.6)
    for x,y,sx,sy in ((25,5,45,3),(45,25,3,40),(25,43,40,3)):
        box(ax,x,y,.025,sx,sy,.05,'#626e70')
    box(ax,2,2,.06,2.5,2.5,.12,'#436b75')
    for z in (.2,3,5.7): box(ax,24,23.5,z,12,13,.35,'#9aa3a2',.75)
    for x in (18.5,24,29.5):
        for y in (17.5,23.5,29.5):
            box(ax,x,y,2.9,.42,.42,5.8,'#7d8788')
    for x in (32.5,37.5):
        for y in (31.5,38.5): box(ax,x,y,2.4,.11,.11,4.8,'#536673')
    for z in (1.3,2.6,4):
        for y in (31.5,38.5): box(ax,35,y,z,5.1,.09,.09,'#536673')
    box(ax,35,35,2.6,5.1,7.1,.1,'#866e52')
    box(ax,11,33,1.35,8,4,2.7,'#40688a')
    for x in (9,11,13): box(ax,x,9.5,.35,1.3,1.5,.7,'#b6aba0')
    box(ax,11,20.5,.015,6,7,.03,'#6c5445')
    for y in (16.6,24.4): box(ax,11,y,.55,7,.13,1.1,'#e76d28')
    box(ax,40.5,12.5,4.5,.7,.7,9,'#e2bd38')
    box(ax,36,12.5,9.1,13,.42,.42,'#e2bd38')
    box(ax,41.8,12.5,8.6,1.4,1.4,.9,'#536673')
    ax.set(xlim=(0,50),ylim=(0,50),zlim=(0,12),xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)')
    ax.set_box_aspect((1,1,.25), zoom=.86)
    ax.view_init(elev=37, azim=-67)
    ax.set_facecolor('#eff4f4')
    for axis in (ax.xaxis,ax.yaxis,ax.zaxis):
        axis.pane.fill = False
        axis.line.set_color('#89999c')


def frame(target_s, label):
    index = int(np.argmin(abs(samples[:,0]-target_s)))
    t,x,y,z = samples[index]
    fig = plt.figure(figsize=(16,9), facecolor='#f8faf9')
    ax = fig.add_axes([.02,.12,.65,.77], projection='3d')
    site_3d(ax)
    trail = samples[:index+1:3]
    ax.plot(trail[:,1],trail[:,2],trail[:,3],color='#0098a0',lw=3.2,alpha=.93)
    ax.scatter([x],[y],[z],s=250,c='#d93629',edgecolors='white',linewidths=2,depthshade=False)
    ax.plot([x,x],[y,y],[0,z],color='#d93629',ls='--',lw=1.3)
    ax.text(x,y,z+1.0,'DRONE',color='#a5231a',fontsize=10,weight='bold')

    map_ax = fig.add_axes([.68,.24,.29,.57])
    map_ax.set_xlim(0,50); map_ax.set_ylim(0,50); map_ax.set_aspect('equal')
    map_ax.set_facecolor('#d9d0bb')
    for a,b,c,d in OBSTACLES:
        map_ax.add_patch(Rectangle((a,c),b-a,d-c,facecolor='#70818a',
                                   edgecolor='#42545b',alpha=.75))
    map_ax.plot(samples[:,1],samples[:,2],color='#ffffff',lw=1.6,alpha=.8,
                label='complete route')
    map_ax.plot(samples[:index+1,1],samples[:index+1,2],color='#007d86',lw=3,
                label='flown by this frame')
    wp=np.array(WAYPOINTS)
    map_ax.scatter(wp[:,0],wp[:,1],s=19,c='#f3c44a',edgecolors='#765a1b',zorder=4)
    map_ax.scatter([x],[y],s=170,c='#d93629',edgecolors='white',linewidths=1.5,zorder=5)
    map_ax.add_patch(Circle((x,y),.325,fill=False,edgecolor='#b52820',linewidth=1.3,zorder=6))
    map_ax.grid(alpha=.18)
    map_ax.set(xlabel='X (m)',ylabel='Y (m)',title='Top-down route and site footprints')
    map_ax.legend(loc='lower right',fontsize=8,framealpha=.9)

    fig.text(.035,.943,'X650 CONSTRUCTION-SITE FLIGHT',fontsize=23,weight='bold',color='#183e45')
    fig.text(.035,.904,f'{label}  •  t = {t:.1f} s  •  position = ({x:.1f}, {y:.1f}, {z:.1f}) m',
             fontsize=12,color='#34575d')
    fig.text(.035,.035,'Isaac Sim telemetry reconstruction · procedural site geometry · highlighted drone marker is enlarged for visibility',
             fontsize=9,color='#5c6e72')
    path=OUT/f'frame_{int(target_s):03d}s.png'
    fig.savefig(path,dpi=130,facecolor=fig.get_facecolor())
    plt.close(fig)
    print(path)
    return path


paths=[frame(t,label) for t,label in ((15,'South access road'),(30,'Passing the building'),
                                      (45,'North road and scaffold'),(60,'East road near crane'))]

from PIL import Image, ImageOps, ImageDraw
thumbs=[]
for p in paths:
    im=Image.open(p).convert('RGB')
    im.thumbnail((960,540))
    thumbs.append(im)
sheet=Image.new('RGB',(1920,1080),'#f8faf9')
for i,im in enumerate(thumbs):
    sheet.paste(im,((i%2)*960,(i//2)*540))
sheet_path=OUT/'storyboard.png'
sheet.save(sheet_path)
print(sheet_path)
