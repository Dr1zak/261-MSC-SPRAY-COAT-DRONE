"""Structural, SI-unit, inertia, local-mesh and kinematic checks; no simulator required."""
from pathlib import Path
import json,math,struct,xml.etree.ElementTree as E
import numpy as np
H=Path(__file__).resolve().parent;p=H/'urdf/x500_v2_sprayer.urdf';r=E.parse(p).getroot()
links={l.attrib['name']:l for l in r.findall('link')};joints=r.findall('joint');parents={};mesh_count=0;mass_sum=0;min_eigen=1e9
assert len(links)==len(r.findall('link'))
for j in joints:
    parent=j.find('parent').attrib['link'];child=j.find('child').attrib['link']
    assert parent in links and child in links and child not in parents
    parents[child]=parent
    if j.attrib['type']=='revolute':
        axis=np.fromstring(j.find('axis').attrib['xyz'],sep=' ');assert abs(np.linalg.norm(axis)-1)<1e-10
        limit=j.find('limit').attrib;assert float(limit['lower'])<float(limit['upper'])
        assert float(limit['velocity'])>0 and float(limit['effort'])>0
assert set(links)-set(parents)=={'base_link'}
for name in links:
    seen=set();n=name
    while n in parents:assert n not in seen;seen.add(n);n=parents[n]
    assert n=='base_link'
for link in links.values():
    for mesh in link.findall('.//mesh'):
        q=(p.parent/mesh.attrib['filename']).resolve();assert q.is_file(),q
        assert mesh.attrib['scale']=='0.001 0.001 0.001';mesh_count+=1
    i=link.find('inertial')
    if i is None:assert link.attrib['name']=='spray_tip';continue
    m=float(i.find('mass').attrib['value']);assert m>0;mass_sum+=m
    a=i.find('inertia').attrib
    M=np.array([[float(a['ixx']),float(a['ixy']),float(a['ixz'])],[float(a['ixy']),float(a['iyy']),float(a['iyz'])],[float(a['ixz']),float(a['iyz']),float(a['izz'])]])
    eig=np.linalg.eigvalsh(M);assert eig.min()>0 and eig[2]<=eig[0]+eig[1]+1e-10
    min_eigen=min(min_eigen,eig.min())
def xyz(j):return np.fromstring(j.find('origin').attrib['xyz'],sep=' ')
js={j.attrib['name']:j for j in joints};T=xyz(js['sprayer_mount_fixed']);N=xyz(js['nozzle_yaw']);tip=xyz(js['spray_tip_fixed'])
tip_positions=[]
for theta in np.linspace(-math.pi/2,0,19):
    c,s=math.cos(theta),math.sin(theta);R=np.array([[c,-s,0],[s,c,0],[0,0,1]])
    q=T+N+R@tip;assert np.allclose(q,[.075,-.01972409,-.240]);tip_positions.append(q.tolist())
# Crank roller centre starts15mm to the left of the servo axis.
th=float(js['can_trigger'].find('limit').attrib['lower']);drop=.015*math.sin(th)
assert abs(drop+.003)<1e-10
stats=json.loads((H/'assembly_data.json').read_text());assert not stats['landing_leg_proxy_clashes']
report=dict(xml_valid=True,links=len(links),joints=len(joints),root='base_link',revolute_joints=[j.attrib['name'] for j in joints if j.attrib['type']=='revolute'],mesh_references_resolved=mesh_count,all_inertias_positive_definite=True,inertia_triangle_inequality_passed=True,mass_sum_kg=mass_sum,minimum_inertia_eigenvalue_kg_m2=float(min_eigen),sampled_nozzle_poses=19,spray_tip_xyz_m=tip_positions[0],modeled_trigger_roller_drop_mm=drop*1000,urdf_lengths='metres; source mesh millimetres with0.001 scale',simulator_import_tested=False,structural_loads_tested=False,flexible_hose_dynamics_tested=False,flight_model_validated=False)
(H/'urdf_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
