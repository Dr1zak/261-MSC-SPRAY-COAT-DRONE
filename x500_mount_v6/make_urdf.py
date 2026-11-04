"""Generate portable URDF from supplied base_link and CAD-derived mechanism frames.
Python3 + NumPy. Meshes remain millimetres; all URDF numbers are SI.
"""
from pathlib import Path
import json,math,csv,xml.etree.ElementTree as E
import numpy as np
H=Path(__file__).resolve().parent
a=json.loads((H/'assembly_data.json').read_text())
f=lambda v:' '.join(f'{float(x):.10g}' for x in v)
def sub(p,t,**kw):return E.SubElement(p,t,{k:str(v) for k,v in kw.items()})
def origin(p,xyz=(0,0,0),rpy=(0,0,0)):sub(p,'origin',xyz=f(xyz),rpy=f(rpy))
root=E.Element('robot',name='x500_v2_compact_sprayer_v6')
root.append(E.Comment('DIMENSIONAL / KINEMATIC PROTOTYPE. Masses and inertias are estimates; not a validated flight model.'))
base=E.parse(H/'reference/x500_v2_original.urdf').getroot().find("link[@name='base_link']")
assert base is not None
for c in list(base.findall('collision')):base.remove(c)
base.find('.//mesh').set('filename','../meshes/x500_frame.stl')
base.insert(0,E.Comment('Original inertial values retained:0.610kg FRAME KIT ONLY. Complete-aircraft mass is unknown. Original solid-box collider replaced by sparse geometry proxies.'))
root.append(base)
def collision(link,name,kind,size,xyz,rpy=(0,0,0)):
    c=sub(link,'collision',name=name);origin(c,xyz,rpy);geo=sub(c,'geometry')
    if kind=='box':sub(geo,'box',size=f(size))
    else:sub(geo,'cylinder',radius=size[0],length=size[1])
def rod(link,name,p1,p2,r):
    p1=np.array(p1)/1000;p2=np.array(p2)/1000;d=p2-p1;l=np.linalg.norm(d)
    # Rz(yaw) Ry(pitch) rotates the cylinder's local Z onto the line.
    yaw=math.atan2(d[1],d[0]);pitch=math.acos(d[2]/l)
    collision(link,name,'cylinder',[r/1000,l],(p1+p2)/2,[0,pitch,yaw])
cx=-1.012093;cy=.27591
for z in (30.809569,60.809569):collision(base,'frame_plate_'+str(z),'box',[.14372,.14372,.002],[cx/1000,cy/1000,z/1000])
for sx,sy in [(-1,-1),(-1,1),(1,-1),(1,1)]:
    rod(base,f'arm_{sx}_{sy}',[cx+sx*53,cy+sy*53,44],[cx+sx*165,cy+sy*165,44],8)
    collision(base,f'motor_{sx}_{sy}','cylinder',[.014,.040],[(cx+sx*175)/1000,(cy+sy*175)/1000,.061])
for y in (-29.724089,30.275911):rod(base,'payload_rail_'+str(y),[-121.012093,y,16.809569],[128.98790,y,16.809569],5)
for ys,ye in [(-56,-119.67759),(56.55,120.22942)]:
    rod(base,'landing_leg_'+str(ys),[cx,ys,23],[cx,ye,-169],8)
    rod(base,'landing_skid_'+str(ys),[-125.012093,ye,-175.966858],[122.987907,ye,-175.966858],5)

mass_report={}
def inertial(link,items):
    masses=np.array([p['mass_kg'] for p in items]);pos=np.array([p['com_mm'] for p in items])/1000
    total=masses.sum();com=(pos*masses[:,None]).sum(0)/total;I=np.zeros((3,3))
    for m,p,x in zip(masses,pos,items):
        w,d,h=np.array(x['size_mm'])/1000
        I+=np.diag(m/12*np.array([d*d+h*h,w*w+h*h,w*w+d*d]))
        r=p-com;I+=m*(np.eye(3)*np.dot(r,r)-np.outer(r,r))
    # Positive-definite check below catches bad physical units or missing dimensions.
    assert np.linalg.eigvalsh(I).min()>0
    e=sub(link,'inertial');origin(e,com);sub(e,'mass',value=f'{total:.10g}')
    sub(e,'inertia',ixx=I[0,0],ixy=I[0,1],ixz=I[0,2],iyy=I[1,1],iyz=I[1,2],izz=I[2,2])
    mass_report[link.attrib['name']]=dict(mass_kg=float(total),com_m=com.tolist(),inertia_kg_m2=I.tolist())
links={}
for name,meshes in a['mesh_groups'].items():
    link=sub(root,'link',name=name);links[name]=link
    link.append(E.Comment('Estimated inertial parameters: see mass_report.json and build_mount.py.'))
    items=[p for p in a['mass_items'] if p['link']==name];inertial(link,items)
    for i,m in enumerate(meshes):
        v=sub(link,'visual',name=f'{name}_visual_{i}');origin(v);geo=sub(v,'geometry')
        sub(geo,'mesh',filename='../meshes/'+m['file'],scale='0.001 0.001 0.001')
        mat=sub(v,'material',name=f'{name}_material_{i}');sub(mat,'color',rgba=f(list(m['color'])+[1]))
    # Sparse collision proxies. Keep holes and pressure components out of collider approximations.
    if name=='sprayer_body':
        collision(link,'can','cylinder',[.03285,.185],[-.04,0,.1045])
        collision(link,'electronics','box',[.094,.041,.062],[-.033,.0615,.096])
        collision(link,'bottom_plate','box',[.178,.09,.004],[.005,-.005,.010])
        collision(link,'spine','box',[.004,.030,.220],[-.079,-.014,.122])
        collision(link,'top_plate','box',[.09,.096,.004],[-.043,0,.234])
    elif name=='nozzle_head':
        collision(link,'valve','box',[.03,.022,.067],[0,.042,.0635])
        collision(link,'dry_shaft','cylinder',[.0075,.085],[0,0,-.0145])
        collision(link,'nozzle','cylinder',[.01,.022],[0,0,-.062])
    elif name=='trigger_crank':collision(link,'horn','box',[.019,.005,.004],[-.0075,0,0])
    elif name=='rail_mount':collision(link,'bridge','box',[.156,.104,.006],[-.007,.00027591,-.023])
    elif name=='landing_risers':
        for i,x in enumerate((-.089012093,.086987907)):
            for j,y in enumerate((-.11967759,.12022942)):
                collision(link,f'foot_{i}_{j}','box',[.05,.038,.006],[x,y,-.276])
                collision(link,f'post_{i}_{j}','box',[.014,.018,.09],[x,y,-.228])
def joint(name,parent,child,xyz,kind='fixed',axis=None,limits=None):
    j=sub(root,'joint',name=name,type=kind);sub(j,'parent',link=parent);sub(j,'child',link=child);origin(j,xyz)
    if axis:sub(j,'axis',xyz=f(axis))
    if limits:
        sub(j,'limit',lower=limits[0],upper=limits[1],effort=1,velocity=1)
        sub(j,'dynamics',damping=.01,friction=0)
        j.append(E.Comment('Effort1Nm and velocity1rad/s are simulation placeholders, not DS3240 specifications.'))
joint('rail_mount_fixed','base_link','rail_mount',[0,0,0])
joint('landing_risers_fixed','base_link','landing_risers',[0,0,0])
joint('sprayer_mount_fixed','rail_mount','sprayer_body',np.array(a['module_translation_mm'])/1000)
joint('nozzle_yaw','sprayer_body','nozzle_head',[.035,-.020,.095],'revolute',[0,0,1],[-math.pi/2,0])
joint('can_trigger','sprayer_body','trigger_crank',[-.025,-.008,.214],'revolute',[0,1,0],[-math.asin(3/15),0])
tip=sub(root,'link',name='spray_tip')
joint('spray_tip_fixed','nozzle_head','spray_tip',[0,0,-.073])
root.append(E.Comment('spray_tip local -Z is discharge direction. No spray, wind, thrust, propeller or fluid simulator is included.'))
E.indent(root,space='  ');out=H/'urdf/x500_v2_sprayer.urdf';E.ElementTree(root).write(out,encoding='utf-8',xml_declaration=True)

# System estimate in the original base_link frame at zero joint position.
trans={'rail_mount':np.zeros(3),'landing_risers':np.zeros(3),'sprayer_body':np.array(a['module_translation_mm'])/1000}
trans['nozzle_head']=trans['sprayer_body']+np.array([.035,-.020,.095]);trans['trigger_crank']=trans['sprayer_body']+np.array([-.025,-.008,.214])
payload=sum(p['mass_kg'] for p in mass_report.values());weighted=np.zeros(3)
for k,p in mass_report.items():weighted+=p['mass_kg']*(np.array(p['com_m'])+trans[k])
mass_report['summary']=dict(added_payload_estimate_kg=payload,supplied_base_mass_kg=.610,modeled_total_estimate_kg=payload+.610,estimated_payload_COM_m=(weighted/payload).tolist(),estimated_system_COM_m=(weighted/(payload+.610)).tolist(),basis='Mixed geometric estimates and assumed purchased-part masses. Excludes unmodeled aircraft battery/avionics and omitted mounting screws. Not a measured takeoff mass.')
(H/'mass_report.json').write_text(json.dumps(mass_report,indent=2))
with (H/'mass_breakdown.csv').open('w',newline='') as fd:
    wr=csv.DictWriter(fd,fieldnames=['name','link','mass_kg','com_mm','size_mm','basis']);wr.writeheader();wr.writerows(a['mass_items'])
(H/'urdf/joint_positions.json').write_text(json.dumps(dict(units='radians',released_horizontal={'nozzle_yaw':0,'can_trigger':0},released_vertical={'nozzle_yaw':-math.pi/2,'can_trigger':0},trigger_pressed={'can_trigger':-math.asin(.2)},note='Set servo mechanical zero and travel using actual hardware. Release trigger before closing solenoid. Controller/interlocks not supplied.'),indent=2))
print(json.dumps(mass_report['summary'],indent=2));print(out)
