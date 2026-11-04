from pathlib import Path
import shutil,json,zipfile,hashlib
H=Path(__file__).resolve().parent;S=H.parent/'compact_hover_v5'
target=H/'source_v5';target.mkdir(exist_ok=True)
for n in ['Compact_Hover_V5_PROVISIONAL.FCStd','geometry_report.json','render_geometry.json','cad_render.py']:
    shutil.copy2(S/n,target/n)
shutil.copytree(S/'stl_FIT_CHECK_ONLY',H/'stl_V5_FIT_CHECK_ONLY',dirs_exist_ok=True)
assert len(list((H/'stl_FIT_CHECK_ONLY').glob('*.stl')))==17
assert len(list((H/'stl_V5_FIT_CHECK_ONLY').glob('*.stl')))==14
assert json.loads((H/'urdf_validation.json').read_text())['xml_valid']
assert not json.loads((H/'cad_validation.json').read_text())['unexpected_adapter_source_intersections']
basic=['README.md','mass_report.json','mass_breakdown.csv','urdf_validation.json','X500_mounted_overview.png','X500_mount_clearance.png']
full=basic+['assembly_data.json','cad_validation.json','build_mount.py','make_urdf.py','validate_urdf.py','validate_cad.py','render_assembly.py','render_geometry.json','package_outputs.py']
def writezip(name,rootfiles,folders):
    selected=[H/n for n in rootfiles]
    for folder in folders:
        selected += [p for p in (H/folder).rglob('*') if p.is_file() and not any(q in p.parts for q in ['__pycache__']) and p.suffix.lower() not in ['.fcstd1','.fcbak','.pyc']]
    # Deduplicate without changing paths. All sources remain within the artifact directory.
    selected=sorted(set(selected));out=H.parent/name
    with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in selected:z.write(p,'x500_mount_v6/'+p.relative_to(H).as_posix())
    with zipfile.ZipFile(out) as z:assert z.testzip() is None
    return dict(file=out.name,files=len(selected),bytes=out.stat().st_size,sha256=hashlib.sha256(out.read_bytes()).hexdigest())
result=[writezip('X500_Sprayer_V6_URDF.zip',basic,['urdf','meshes']),writezip('X500_Sprayer_V6_CAD_and_Source.zip',full,['urdf','meshes','cad','stl_FIT_CHECK_ONLY','stl_V5_FIT_CHECK_ONLY','source_v5','reference'])]
(H/'package_manifest.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
