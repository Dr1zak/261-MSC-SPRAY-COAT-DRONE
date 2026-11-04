#!/usr/bin/env python3
"""Audit URDF, visual meshes, and the source STEP part-to-link mapping.

This validates provenance; USD import runs separately in Isaac's Python.
"""
import hashlib
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/'config/asset_mapping.yaml'


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):
            h.update(block)
    return h.hexdigest()


def product_labels(path):
    return set(re.findall(r"PRODUCT\('([^']*)'",path.read_text(errors='replace')))


def main():
    cfg=yaml.safe_load(CONFIG.read_text())
    paths={key:ROOT/cfg[key] for key in ('urdf','aircraft_step','payload_step','attachment_step','part_mapping_source','aircraft_visual_source')}
    for path in paths.values():
        if not path.is_file():raise FileNotFoundError(path)
    urdf=paths['urdf'];root=ET.parse(urdf).getroot()
    links={e.get('name') for e in root.findall('link')}
    children={j.find('child').get('link') for j in root.findall('joint')}
    roots=sorted(links-children)
    if len(roots)!=1:
        raise ValueError(f'expected one URDF root, found {roots}')
    mesh_map={}
    for link in root.findall('link'):
        meshes=[]
        for mesh in link.findall('./visual/geometry/mesh'):
            name=mesh.get('filename')
            path=(urdf.parent/name).resolve()
            if not path.is_file(): raise FileNotFoundError(path)
            if mesh.get('scale')!='0.001 0.001 0.001':raise ValueError(f'unexpected mesh scale: {path}')
            meshes.append({'path':str(path.relative_to(ROOT)),'scale':mesh.get('scale','1 1 1')})
        mesh_map[link.get('name')]=meshes
    assembly=json.loads(paths['part_mapping_source'].read_text())
    source_map={p['name']:p['link'] for p in assembly['mass_items']}
    explicit_map=cfg['step_part_to_urdf_link']
    if source_map!=explicit_map or len(source_map)!=len(assembly['mass_items']):
        raise ValueError('explicit mapping differs from assembly_data.json')
    if not set(explicit_map.values())<=links:raise ValueError('mapping references unknown URDF link')
    payload_names=product_labels(paths['payload_step'])
    aircraft_names=product_labels(paths['aircraft_step'])
    if not set(explicit_map)<=payload_names or not set(explicit_map)<=aircraft_names:
        raise ValueError('mapped names missing from STEP assembly')
    for group_link,groups in assembly['mesh_groups'].items():
        for group in groups:
            if any(source_map[name]!=group_link for name in group['parts']):
                raise ValueError(f'mesh group has wrong link: {group}')
            if not (ROOT/'x500_mount_v6/meshes'/group['file']).is_file():
                raise FileNotFoundError(group['file'])
    for key in ('aircraft_step','payload_step','attachment_step'):
        with paths[key].open('rb') as stream:header=stream.read(1024*1024)
        if b'SI_UNIT(.MILLI.,.METRE.)' not in header:
            raise ValueError(f'STEP units not confirmed as millimeters: {paths[key]}')
    files=sorted(set(paths.values())|{ROOT/m['path'] for meshes in mesh_map.values() for m in meshes}|{CONFIG})
    manifest={'urdf_root':roots[0],'urdf_robot':root.get('name'),
              'source_sha256':{str(p.relative_to(ROOT)):digest(p) for p in files},
              'urdf_visual_meshes':mesh_map,'step_part_to_urdf_link':explicit_map,
              'step_product_counts':{'aircraft':len(aircraft_names),'payload':len(payload_names)},
              'mapped_step_parts':len(explicit_map),
              'unmapped_aircraft_step_products':sorted(aircraft_names-set(explicit_map)),
              'conversion_status':'validated source mapping; USD import not yet executed',
              'visual_strategy':'Isaac URDF importer reads the supplied link-local STL visuals; STEP names audit provenance, not runtime tessellation',
              'units':'URDF meters; STEP and STL millimeters; STL visual scale 0.001'}
    out=ROOT/'assets/generated/asset_manifest.json'; out.parent.mkdir(parents=True,exist_ok=True)
    if out.exists():
        previous=json.loads(out.read_text())
        if previous.get('source_sha256')==manifest['source_sha256'] and 'isaac_import' in previous:
            usd=ROOT/previous['isaac_import']['usd_path']
            if usd.is_file():
                manifest['isaac_import']=previous['isaac_import']
                manifest['conversion_status']=previous['conversion_status']
    out.write_text(json.dumps(manifest,indent=2,sort_keys=True)+'\n')
    print(f'Validated {len(explicit_map)} STEP mappings and {sum(map(len,mesh_map.values()))} visual meshes; manifest: {out}')
    return 0


if __name__=='__main__':
    try:raise SystemExit(main())
    except (FileNotFoundError,ValueError,KeyError) as exc:
        print(f'asset validation failed: {exc}',file=sys.stderr)
        raise SystemExit(2)
