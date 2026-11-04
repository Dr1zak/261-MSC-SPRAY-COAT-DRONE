"""Build V6 rail-mount CAD and URDF geometry. Run with FreeCAD 0.21 Python.
All CAD dimensions mm. Fit-check prototype; aircraft strength/payload not qualified.
"""
from pathlib import Path
import json,math,shutil
import FreeCAD as A
import Part,Mesh,MeshPart,Import
V=A.Vector; H=Path(__file__).resolve().parent
S=H/'source_v5' if (H/'source_v5').exists() else H.parent/'compact_hover_v5'
for p in ['cad','stl_FIT_CHECK_ONLY','meshes','urdf']: (H/p).mkdir(exist_ok=True)
source=A.openDocument(str(S/'Compact_Hover_V5_PROVISIONAL.FCStd'))
doc=A.newDocument('X500SprayerV6')
palette={p['name']:p for p in json.loads((S/'render_geometry.json').read_text())}
g=json.loads((S/'geometry_report.json').read_text())
manufactured={p['part']:p['mass_kg_solid'] for p in g['manufactured_parts']}
T=V(40,.27591,-262)
nozzle=V(35,-20,95); trigger=V(-25,-8,214)
moving=set(g['moving_parts']); triggers={'TRIGGER_METAL_HORN','TRIGGER_ROLLER'}
parts=[]; renders=[]; new=[]; groups={}; massitems=[]
METAL=(.69,.74,.79); ORANGE=(.92,.43,.09)
def box(x,y,z,w,d,h): return Part.makeBox(w,d,h,V(x,y,z))
def cylx(x,y,z,r,l): return Part.makeCylinder(r,l,V(x,y,z),V(1,0,0))
def cylz(x,y,z,r,l): return Part.makeCylinder(r,l,V(x,y,z))
def obj(name,s,color,kind,link,mass,local_shift=V(0,0,0)):
    assert s.isValid() and s.Volume>0,name
    o=doc.addObject('PartDesign::Feature',name);o.Label=name;o.Shape=s
    o.addProperty('App::PropertyString','Qualification');o.Qualification=kind
    o.addProperty('App::PropertyFloat','EstimatedMassKg');o.EstimatedMassKg=mass
    parts.append(o)
    v,f=s.tessellate(.5)
    renders.append(dict(name=name,color=color,vertices=[[p.x,p.y,p.z] for p in v],faces=f))
    local=s.copy();local.translate(-local_shift)
    groups.setdefault(link,[]).append((name,local,color,mass))
    b=local.BoundBox;c=V(0,0,0)
    for solid in local.Solids:c+=solid.CenterOfMass*solid.Volume/local.Volume
    massitems.append(dict(name=name,link=link,mass_kg=mass,com_mm=[c.x,c.y,c.z],size_mm=[b.XLength,b.YLength,b.ZLength],basis=kind))
    return o
def newpart(name,s,material='aluminum',link='rail_mount',dry=True):
    rho=2.7 if material=='aluminum' else 1.25
    o=obj(name,s,METAL if material=='aluminum' else ORANGE,'FIT CHECK: '+material+'; nominal solid-density mass, strength unverified',link,s.Volume*rho/1e6)
    if dry:
        new.append(o)
        MeshPart.meshFromShape(Shape=s,LinearDeflection=.08,AngularDeflection=.18,Relative=False).write(str(H/'stl_FIT_CHECK_ONLY'/(name+'.stl')))
    return o

# Rail centres measured from supplied STL. STL OD9.4; official rail nominal OD10.
ry=[-29.724089,30.275911]; rz=16.809569; xs=[-74,60]
mount=box(-85,-51.72409,-26,156,104,6)
# Central window preserves load bridges along x and around the four module bolts.
mount=mount.cut(box(-15,-17.72409,-27,30,36,8))
for x in (-37,31):
    for y in (-33.72409,34.27591): mount=mount.cut(cylz(x,y,-27,2.2,9))
for y in (-19.72409,-7.72409): mount=mount.cut(cylz(-40,y,-27,3.2,9))
for i,x in enumerate(xs):
    for j,y in enumerate(ry):
        # Two M3 fasteners on each side of the rail; short upper and lower screws.
        for yy in (y-7.5,y+7.5):
            mount=mount.cut(cylz(x,yy,-27,1.7,9))
            mount=mount.cut(Part.makeCone(3.4,1.7,1.7,V(x,yy,-26)))
        lower=box(x-4.5,y-14,-20,9,28,36.409569)
        lower=lower.cut(cylx(x-5.5,y,rz,5.1,11))
        lower=lower.cut(box(x-5.5,y-5,-18,11,10,22))
        cap=box(x-4.5,y-14,rz+.4,9,28,7.6).cut(cylx(x-5.5,y,rz,5.1,11))
        if i==1:
            # Relief at the outer corner clears the existing frame screw ends.
            cap=cap.cut(box(54,39.6,23.1,6.5,7,4)).cut(box(54,-45.5,23.1,6.5,6.5,4))
        for yy in (y-7.5,y+7.5):
            # Tap-drill representations; M3 thread itself not tessellated.
            lower=lower.cut(cylz(x,yy,7,1.25,11)).cut(cylz(x,yy,-21,1.25,13))
            cap=cap.cut(cylz(x,yy,16,1.7,11))
        newpart(f'RAIL_LOWER_{i}_{j}',lower.removeSplitter())
        newpart(f'RAIL_CAP_{i}_{j}',cap.removeSplitter())
newpart('X500_BRIDGE_156x104',mount.removeSplitter())
obsolete=H/'stl_FIT_CHECK_ONLY/X500_BRIDGE_120x104.stl'
if obsolete.exists():obsolete.unlink()

# Four tall skid shoes: geometric landing-clearance proposal, NOT a qualified gear design.
# Existing skid centreline inferred from full STL bounds, nominal diameter10mm.
for i,x in enumerate((-89.012093,86.987907)):
    for j,y in enumerate((-119.67759,120.22942)):
        z=-175.966858
        lower=box(x-10,y-16,z-10,20,32,9.6).cut(cylx(x-11,y,z,5.1,22))
        lower=lower.fuse(box(x-7,y-9,-273,14,18,90))
        lower=lower.fuse(box(x-25,y-19,-279,50,38,6))
        # Gussets broaden the foot-to-column transition.
        for side in (-1,1):
            pts=[V(x+side*7,y-9,-245),V(x+side*22,y-9,-273),V(x+side*7,y-9,-273)]
            face=Part.Face(Part.makePolygon(pts+[pts[0]]));lower=lower.fuse(face.extrude(V(0,18,0)))
        cap=box(x-10,y-16,z+.4,20,32,8).cut(cylx(x-11,y,z,5.1,22))
        for yy in (y-12,y+12):
            lower=lower.cut(cylz(x,yy,z-11,1.25,12));cap=cap.cut(cylz(x,yy,z,1.7,12))
        newpart(f'OPTIONAL_SKID_RISER_{i}_{j}',lower.removeSplitter(),'polymer','landing_risers')
        newpart(f'OPTIONAL_SKID_CAP_{i}_{j}',cap.removeSplitter(),'polymer','landing_risers')

# Use explicitly provisional purchased-part masses, never density of solid envelopes.
servo_volume={p:sum(o.Shape.Volume for o in source.Objects if o.Label.startswith(p)) for p in ['DS3240_TRIGGER','DS3240_ORIENTATION']}
valve_volume=sum(o.Shape.Volume for o in source.Objects if o.Label.startswith(('VALVE_2V025','VALVE_TOP_NUT')))
pcb_volume=sum(o.Shape.Volume for o in source.Objects if o.Label.startswith(('PCB_','ELECTRONICS_TERMINAL')))
def mass_for(n,s):
    if n in manufactured:return manufactured[n],'V5 solid-density structural estimate'
    if n=='CAN_OD65_7_HEIGHT_ASSUMED':return .45,'ASSUMED full can450g; replace with scale measurement'
    for p in servo_volume:
        if n.startswith(p):return .072*s.Volume/servo_volume[p],'ASSUMED servo72g each; replace with actual version'
    if n.startswith(('VALVE_2V025','VALVE_TOP_NUT')):return .20*s.Volume/valve_volume,'ASSUMED complete valve200g'
    if n.startswith(('PCB_','ELECTRONICS_TERMINAL')):return .08*s.Volume/pcb_volume,'ASSUMED electronics80g total'
    if n.startswith('BEARING_') and n!='BEARING_HOUSING_RETENTION_BAND':return .029,'ASSUMED bearing29g each'
    if n.startswith('HOSE_'):return s.Volume*1.3/1e6,'ASSUMED hose density1.3g/cm3; fluid excluded'
    rho=1.25 if n.startswith(('WIRE_','TIMING_','TRIGGER_ROLLER')) else 2.7
    return max(.0001,s.Volume*rho/1e6),'APPROXIMATE solid-envelope mass at '+str(rho)+'g/cm3'
for o in source.Objects:
    if not hasattr(o,'Shape'):continue
    n=o.Label;s=o.Shape.copy();m,basis=mass_for(n,s);s.translate(T)
    link='nozzle_head' if n in moving else 'trigger_crank' if n in triggers else 'sprayer_body'
    origin=T+(nozzle if link=='nozzle_head' else trigger if link=='trigger_crank' else V(0,0,0))
    obj(n,s,palette[n]['color'],basis,link,m,origin)

# Original drone stays a mesh: converting860k triangles to STEP is intentionally avoided.
drone=doc.addObject('Mesh::Feature','X500_USER_MESH');drone.Label='X500 supplied mesh - mm';drone.Mesh=Mesh.Mesh(str(H/'reference/x500_frame.stl'))
shutil.copy2(H/'reference/x500_frame.stl',H/'meshes/x500_frame.stl')
doc.recompute();doc.saveAs(str(H/'cad/X500_Sprayer_V6_FIT_CHECK.FCStd'))
Import.export(new,str(H/'cad/X500_Rail_Mount_and_Risers_FIT_CHECK.step'))
Import.export(parts,str(H/'cad/Sprayer_with_X500_Mount_FIT_CHECK.step'))

# Local-frame, material-grouped URDF meshes (mm, converted using URDF scale).
meshinfo={}
for link,items in groups.items():
    colors={}
    for n,s,c,m in items:
        # Flexible upstream hose cannot follow a revolute joint as a rigid mesh.
        # Retain its mass estimate, but show it only in nominal-pose CAD renders.
        if n=='HOSE_SUPPLY_SERVICE_LOOP_0deg':continue
        colors.setdefault(tuple(c),[]).append((n,s))
    meshinfo[link]=[]
    for i,(c,ss) in enumerate(colors.items()):
        name=f'{link}_{i:02d}.stl';compound=Part.makeCompound([s for n,s in ss])
        MeshPart.meshFromShape(Shape=compound,LinearDeflection=.18,AngularDeflection=.25,Relative=False).write(str(H/'meshes'/name))
        meshinfo[link].append(dict(file=name,color=c,parts=[n for n,s in ss]))

# Conservative sampled clearances to the source landing legs, using16mm tubes.
legs=[Part.makeCylinder(8,(V(-1.012093,yend,-169)-V(-1.012093,ystart,23)).Length,V(-1.012093,ystart,23),(V(-1.012093,yend,-169)-V(-1.012093,ystart,23)).normalize()) for ystart,yend in [(-56,-119.67759),(56.55,120.22942)]]
nearest=(1e9,None);clashes=[]
fixed=[o for o in parts if o.Label not in moving and o.Label not in triggers and not o.Label.startswith(('RAIL_','X500_','OPTIONAL_'))]
head=[o for o in parts if o.Label in moving]
for deg in range(0,-91,-5):
    for o in (fixed+head if deg==0 else head):
        if 'HOSE_SUPPLY' in o.Label and deg!=0:continue
        s=o.Shape.copy()
        if o in head:s.rotate(T+nozzle,V(0,0,1),deg)
        for j,leg in enumerate(legs):
            b=s.BoundBox;q=leg.BoundBox
            lower_bound=math.sqrt(sum(max(0,getattr(b,axis+'Min')-getattr(q,axis+'Max'),getattr(q,axis+'Min')-getattr(b,axis+'Max'))**2 for axis in ('X','Y','Z')))
            if lower_bound>nearest[0]:continue
            dist=s.distToShape(leg)[0]
            if dist<nearest[0]:nearest=(dist,[deg,o.Label,j])
            if dist<1e-6:clashes.append([deg,o.Label,j])
bb=Part.makeCompound([o.Shape for o in parts]+[Part.makeBox(400.33,400.33,.01,V(-201.177,-199.889,136.80))]).BoundBox
report=dict(status='FIT-CHECK assembly; unqualified airframe payload, clamp strength and hose motion',module_translation_mm=[T.x,T.y,T.z],rail_centres_mm=[[0,y,rz] for y in ry],rail_nominal_OD_mm=10,rail_mesh_OD_mm=9.4,clamp_bore_mm=10.2,clamp_x_mm=xs,module_lowest_z_mm=-254,stock_ground_z_mm=-185.466858,stock_ground_interference_mm=68.533142,proposed_ground_z_mm=-279,proposed_module_clearance_mm=25,nozzle_outlet_z_mm=-240,proposed_nozzle_clearance_mm=39,landing_leg_proxy_min_clearance_mm=nearest[0],landing_leg_proxy_min_pair=nearest[1],landing_leg_proxy_clashes=clashes,all_new_shapes_valid=all(o.Shape.isValid() for o in new),new_dry_parts=len(new),mesh_groups=meshinfo,mass_items=massitems,source_base_mass_kg=.610,source_base_mass_basis='User URDF: frame-kit mass only; battery/motors/avionics not established',inertia_method='Each part: axis-aligned box inertia at CAD COM, then parallel-axis sum; estimates, not measured')
(H/'assembly_data.json').write_text(json.dumps(report,indent=2))
(H/'render_geometry.json').write_text(json.dumps(renders))
print(json.dumps({k:v for k,v in report.items() if k not in ['mesh_groups','mass_items']},indent=2))
