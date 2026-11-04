from pathlib import Path
import json,sys
import numpy as np
from PIL import Image,ImageDraw,ImageFont
H=Path(__file__).resolve().parent
sys.path.insert(0,str(H/'source_v5' if (H/'source_v5').exists() else H.parent/'compact_hover_v5'))
from cad_render import draw_cad
drone=json.loads((H/'reference/drone_render_merged.json').read_text())
parts=json.loads((H/'render_geometry.json').read_text());a=json.loads((H/'assembly_data.json').read_text())
def font(n,b=False):return ImageFont.truetype('C:/Windows/Fonts/'+('arialbd.ttf' if b else 'arial.ttf'),n)
def txt(d,xy,t,n=24,b=False,c='#16364c'):d.text(xy,t,font=font(n,b),fill=c)
def fit(im,ps,rect,cam):
    c=np.array(cam,dtype=float);c/=np.linalg.norm(c);r=np.cross([0,0,1],c);r/=np.linalg.norm(r);u=np.cross(c,r)
    v=np.concatenate([p['vertices'] for p in ps]);xy=np.stack([v@r,-v@u],1);lo=xy.min(0);hi=xy.max(0)
    x,y,w,h=rect;s=min(w/(hi-lo)[0],h/(hi-lo)[1]);org=np.array([x+w/2,y+h/2])-(lo+hi)*s/2
    return draw_cad(im,ps,org,s,camera_direction=cam)
im=Image.new('RGB',(1800,1320),'#edf3f6');d=ImageDraw.Draw(im)
d.rectangle((0,0,1800,140),fill='#102f46');txt(d,(45,25),'X500 V2 + compact rotating sprayer',46,True,'white')
txt(d,(47,91),'Actual supplied drone mesh | Rail-mounted fit-check assembly + articulated URDF',24,False,'#c5dde8')
project=fit(im,drone+parts,(35,174,1210,1000),(1,-1,.55));d=ImageDraw.Draw(im)
cards=[('1  SPLIT RAIL CLAMPS','Four clamps around existing rods.','10.2 mm bore for nominal 10 mm rails.',(-74,-29.72,22)),
 ('2  UPRIGHT CAN + TRIGGER','65.7 mm can; DS3240 trigger retained.','Assumed can height still needs checking.',(0,-28,-115)),
 ('3  ROTATING SPRAY HEAD','Same valve and belt-driven DS3240.','Nozzle joint: 0 to -90 degrees.',(75,-19.72,-205)),
 ('4  ELECTRONICS BOX','Controller, 6 V supply and valve driver.','Box retained behind the can.',(40,77,-168)),
 ('5  PROPOSED LANDING RISERS','Orange parts are fit-check concepts.','25 mm modeled base-to-ground gap.',(87,-120,-247))]
for i,(title,line1,line2,p) in enumerate(cards):
    y=177+i*188;d.rounded_rectangle((1260,y,1770,y+165),14,fill='white')
    txt(d,(1277,y+18),title,26,True);txt(d,(1277,y+69),line1,21);txt(d,(1277,y+112),line2,20)
    x0,y0=project(np.array(p));d.ellipse((x0-17,y0-17,x0+17,y0+17),fill='#087c95',outline='white',width=2);txt(d,(x0-7,y0-13),str(i+1),22,True,'white')
txt(d,(50,1200),'STOCK LANDING GEAR IS TOO SHORT: module projects 68.5 mm below the original ground plane.',25,True)
txt(d,(50,1250),'Prototype only. Actual payload, clamp strength, riser stability and flexible-hose motion require qualification.',23)
im.save(H/'X500_mounted_overview.png')

im=Image.new('RGB',(1800,1250),'#edf3f6');d=ImageDraw.Draw(im)
d.rectangle((0,0,1800,130),fill='#102f46');txt(d,(45,24),'Rail attachment and ground clearance',44,True,'white')
txt(d,(46,85),'Dimensions are measured/modelled in millimetres. Orange risers are an unqualified clearance proposal.',23,False,'#c5dde8')
small=[p for p in parts if p['name'].startswith(('RAIL_','X500_BRIDGE','03_COMPACT'))]
rail_geometry=json.loads((H/'reference/drone_display_geometry.json').read_text())
small += [p for p in rail_geometry if p['name'] in ('DRONE_PART_231','DRONE_PART_233','DRONE_PART_205','DRONE_PART_206','DRONE_PART_185','DRONE_PART_186')]
fit(im,small,(65,180,820,630),(1,-1,.7))
proj=fit(im,drone+parts,(930,170,810,800),(0,-1,.008));d=ImageDraw.Draw(im)
txt(d,(55,845),'Rail adapter',31,True);txt(d,(55,895),'156 x 104 x 6 mm bridge; M4 holes match V5 slots.',23)
txt(d,(55,940),'Rail centres: 60 mm apart. Clamp engagement: 9 mm each.',23)
txt(d,(55,985),'Machined aluminium; tapped M3 holes. STL is for fit checks.',22)
for z,label,c in [(-185.466858,'Original ground','#bd4a3a'),(-279,'Proposed ground','#227d60')]:
    x0,y0=proj(np.array([-145,0,z]));x1,y1=proj(np.array([150,0,z]));d.line((x0,y0,x1,y1),fill=c,width=3);txt(d,(x0,y0+6),label,21,True,c)
txt(d,(955,1030),'New clearance: base 25 mm; nozzle 39 mm.',24,True)
txt(d,(55,1120),'The source rails are meshed at 9.4 mm OD; nominal X500 rails are 10 mm. Measure your actual rails before machining.',23)
txt(d,(55,1170),'No battery, wiring clearance or clamp torque is inferred from this frame-only STL. No structural or flight test has been performed.',22)
im.save(H/'X500_mount_clearance.png')
print('Rendered two assembly views')
