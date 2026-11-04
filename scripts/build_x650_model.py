#!/usr/bin/env python3
"""Build the X650 sprayer URDF from Holybro frame mesh and retained payload CAD.

The payload bridge is dimensionally aligned to the X650 60 mm rails but its
fastening, exact interference, and strength are not validated.
"""
import hashlib
import json
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'x500_mount_v6/urdf/x500_v2_sprayer.urdf'
DEST = ROOT / 'x650_mount/urdf/x650_sprayer.urdf'
MESH_DIR = DEST.parent.parent / 'meshes'
FRAME = MESH_DIR / 'x650_frame.stl'
if not FRAME.is_file() or FRAME.stat().st_size == 0:
    raise SystemExit('Export the X650 STEP frame first with scripts/export_x650_frame.py')

tree = ET.parse(SOURCE)
robot = tree.getroot()
robot.set('name', 'x650_compact_sprayer')
for node in list(robot):
    if node.tag == 'link' and node.get('name') == 'landing_risers':
        robot.remove(node)
    elif node.tag == 'joint' and node.get('name') == 'landing_risers_fixed':
        robot.remove(node)

base = robot.find("link[@name='base_link']")
for visual in list(base.findall('visual')):
    base.remove(visual)
for collision in list(base.findall('collision')):
    base.remove(collision)


def fmt(values):
    return ' '.join(f'{float(value):.10g}' for value in values)


def origin(parent, xyz, rpy=(0, 0, 0)):
    return ET.SubElement(parent, 'origin', xyz=fmt(xyz), rpy=fmt(rpy))


def geometry(parent, kind, size):
    element = ET.SubElement(parent, 'geometry')
    if kind == 'box':
        ET.SubElement(element, 'box', size=fmt(size))
    elif kind == 'cylinder':
        ET.SubElement(element, 'cylinder', radius=str(size[0]), length=str(size[1]))
    else:
        raise ValueError(kind)


def collision(name, kind, size, xyz, rpy=(0, 0, 0)):
    node = ET.SubElement(base, 'collision', name=name)
    origin(node, xyz, rpy)
    geometry(node, kind, size)


def rod(name, start, end, radius):
    a = np.asarray(start, dtype=float)
    b = np.asarray(end, dtype=float)
    delta = b-a
    length = np.linalg.norm(delta)
    yaw = math.atan2(delta[1], delta[0])
    pitch = math.acos(delta[2]/length)
    collision(name, 'cylinder', (radius, length), (a+b)/2, (0, pitch, yaw))


visual = ET.SubElement(base, 'visual', name='holybro_x650_frame')
origin(visual, (0, 0, 0))
geo = ET.SubElement(visual, 'geometry')
ET.SubElement(geo, 'mesh', filename='../meshes/x650_frame.stl', scale='0.001 0.001 0.001')
material = ET.SubElement(visual, 'material', name='x650_carbon_dark')
ET.SubElement(material, 'color', rgba='0.13 0.16 0.18 1')

collision('central_frame', 'box', (.16, .16, .07), (0, 0, .05))
rotors = [(-.22981,-.22981,.075),(-.22981,.22981,.075),
          (.22981,-.22981,.075),(.22981,.22981,.075)]
for index, (x,y,z) in enumerate(rotors, 1):
    rod(f'x650_arm_{index}', (math.copysign(.055,x),math.copysign(.055,y),.055),
        (x,y,z), .011)
    collision(f'x650_motor_{index}', 'cylinder', (.024, .05), (x,y,z))
    prop = ET.SubElement(base, 'visual', name=f'propeller_swept_disk_{index}')
    origin(prop, (x,y,z+.035))
    geometry(prop, 'cylinder', (.3937/2, .002))
    color = ET.SubElement(prop, 'material', name=f'propeller_swept_disk_color_{index}')
    ET.SubElement(color, 'color', rgba='0.72 0.79 0.81 0.28')
for index, y in enumerate((-.03,.03), 1):
    rod(f'x650_payload_rail_{index}', (-.155,y,.016809569),(.165,y,.016809569),.005)
for index, y in enumerate((-.1589,.1589), 1):
    rod(f'x650_landing_skid_{index}', (-.16,y,-.273190431),(.16,y,-.273190431),.005)
    for side, x in enumerate((-.10,.10), 1):
        rod(f'x650_landing_leg_{index}_{side}',
            (x,math.copysign(.055,y),.015), (x,y,-.265), .008)

# Published kit mass is 2 kg without battery. Distribute the extra 1.39 kg
# from the original 0.61 kg seed to the center and four rotor pods.
seed_mass = .610
items = [(seed_mass, np.array((0.,0.,0.))),
         (.590, np.array((0.,0.,.020)))]
items += [(.200, np.array(point)) for point in rotors]
base_mass = sum(mass for mass, _ in items)
base_com = sum((mass*point for mass, point in items), np.zeros(3))/base_mass
inertia = np.diag((.0125,.0125,.0220))
for mass, point in items:
    offset = point-base_com
    inertia += mass*(np.dot(offset,offset)*np.eye(3)-np.outer(offset,offset))
base_inertial = base.find('inertial')
base_inertial.find('origin').set('xyz', fmt(base_com))
base_inertial.find('mass').set('value', fmt((base_mass,)))
tensor = base_inertial.find('inertia')
for key, value in dict(ixx=inertia[0,0], ixy=inertia[0,1], ixz=inertia[0,2],
                       iyy=inertia[1,1], iyz=inertia[1,2], izz=inertia[2,2]).items():
    tensor.set(key, f'{value:.10g}')

battery = ET.SubElement(robot, 'link', name='battery_6s')
battery_inertial = ET.SubElement(battery, 'inertial')
origin(battery_inertial, (0,0,0))
ET.SubElement(battery_inertial, 'mass', value='0.750')
length,width,height = .15,.055,.05  # provisional dimensions for inertial proxy
battery_inertia = (.750/12*(width**2+height**2),
                   .750/12*(length**2+height**2),
                   .750/12*(length**2+width**2))
ET.SubElement(battery_inertial, 'inertia',
              ixx=str(battery_inertia[0]), ixy='0', ixz='0',
              iyy=str(battery_inertia[1]), iyz='0', izz=str(battery_inertia[2]))
battery_joint = ET.SubElement(robot, 'joint', name='battery_6s_fixed', type='fixed')
ET.SubElement(battery_joint, 'parent', link='base_link')
ET.SubElement(battery_joint, 'child', link='battery_6s')
origin(battery_joint, (0,0,.045))

MESH_DIR.mkdir(parents=True, exist_ok=True)
for link in robot.findall('link'):
    for mesh in link.findall('./visual/geometry/mesh'):
        filename = Path(mesh.get('filename')).name
        if filename != 'x650_frame.stl':
            source = ROOT / 'x500_mount_v6/meshes' / filename
            if not source.is_file():
                raise FileNotFoundError(source)
            shutil.copy2(source, MESH_DIR / filename)
        mesh.set('filename', '../meshes/' + filename)

DEST.parent.mkdir(parents=True, exist_ok=True)
ET.indent(robot, space='  ')
tree.write(DEST, encoding='utf-8', xml_declaration=True)
payload_mass = sum(float(link.find('./inertial/mass').get('value'))
                   for link in robot.findall('link')
                   if link.get('name') not in ('base_link','battery_6s','spray_tip'))
metadata = {
    'model': str(DEST.relative_to(ROOT)),
    'source_frame_step': 'assets/source/x650-frame-2025-0825.stp',
    'source_frame_step_sha256': hashlib.sha256((ROOT/'assets/source/x650-frame-2025-0825.stp').read_bytes()).hexdigest(),
    'frame_mesh': str(FRAME.relative_to(ROOT)),
    'frame_mesh_sha256': hashlib.sha256(FRAME.read_bytes()).hexdigest(),
    'cad_to_model': 'rotate +90 deg about X; translate +29.809569 mm along model Z',
    'base_kit_mass_kg': base_mass,
    'base_com_m': base_com.tolist(),
    'battery_mass_kg': .750,
    'payload_mass_kg': payload_mass,
    'modeled_total_mass_kg': base_mass+.750+payload_mass,
    'rotor_positions_m': rotors,
    'status': 'simulation model; exact mount fit, component masses and thrust curve remain unverified',
}
(DEST.parent.parent/'model_metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
print(json.dumps(metadata,indent=2))
