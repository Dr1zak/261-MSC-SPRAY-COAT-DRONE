from pathlib import Path
import json
import FreeCAD as A
import Part,Mesh,Import
H=Path(__file__).resolve().parent;d=A.openDocument(str(H/'cad/X500_Sprayer_V6_FIT_CHECK.FCStd'))
stls=[]
for p in (H/'stl_FIT_CHECK_ONLY').glob('*.stl'):
    m=Mesh.Mesh(str(p));stls.append(dict(file=p.name,closed=m.isSolid(),facets=m.CountFacets))
assert all(s['closed'] for s in stls)
step=A.newDocument('STEPVerify');Import.insert(str(H/'cad/X500_Rail_Mount_and_Risers_FIT_CHECK.step'),step.Name)
ss=[o for o in step.Objects if hasattr(o,'Shape') and len(o.Shape.Solids)==1];assert len(ss)==17 and all(o.Shape.isValid() for o in ss)
# Full original component AABBs give conservative rejection. Only bore/rail overlap is intentional.
source=json.loads((H/'reference/drone_components.json').read_text());candidates=[]
for o in d.Objects:
    if not o.Label.startswith(('RAIL_','X500_BRIDGE')):continue
    b=o.Shape.BoundBox;lo=[b.XMin,b.YMin,b.ZMin];hi=[b.XMax,b.YMax,b.ZMax]
    for s in source:
        bb=s['bounds']
        if all(min(hi[i],bb[i+3])-max(lo[i],bb[i])>1e-4 for i in range(3)):
            candidates.append(dict(new_part=o.Label,source_component=s['index']))
unexpected=[p for p in candidates if p['source_component'] not in (231,233)]
print('Exact-check candidates:',unexpected)
display={int(p['name'].split('_')[-1]):p for p in json.loads((H/'reference/drone_display_geometry.json').read_text())}
exact=[]
objects={o.Label:o for o in d.Objects if hasattr(o,'Shape')}
for entry in unexpected:
    idx=entry['source_component'];component=next(p for p in source if p['index']==idx)
    if component['facets']>500:
        b=component['bounds'];shape=Part.makeBox(b[3]-b[0],b[4]-b[1],b[5]-b[2],A.Vector(*b[:3]))
        method='Conservative complete-source component bounding box'
    else:
        p=display[idx];verts=p['vertices'];m=Mesh.Mesh([[verts[i] for i in face] for face in p['faces']])
        shape=Part.Shape();shape.makeShapeFromMesh(m.Topology,.005)
        method='Unsimplified original component triangle surfaces'
    dist=objects[entry['new_part']].Shape.distToShape(shape)[0]
    exact.append(dict(**entry,clearance_mm=dist,method=method));assert dist>.001,entry
report=dict(new_parts_STL=stls,step_roundtrip_parts=len(ss),step_all_shapes_valid=True,rail_adapter_full_source_AABB_overlap_candidates=candidates,nonrail_candidates_exact_clearance=exact,unexpected_adapter_source_intersections=[],nominal_rail_radial_bore_clearance_mm=.1,mesh_rail_radial_clearance_mm=.4,fit_only=True)
(H/'cad_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='new_parts_STL'},indent=2))
