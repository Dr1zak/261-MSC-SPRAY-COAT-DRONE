#!/usr/bin/env python3
"""Import the X650 CAD-frame sprayer URDF into Isaac Sim 6.0 USD."""
import argparse
import hashlib
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--workspace', type=Path, required=True)
parser.add_argument('--isaac-root', type=Path, required=True)
args, _ = parser.parse_known_args()
root = args.workspace.resolve()
version = (args.isaac_root / 'VERSION').read_text().strip()
if not version.startswith('6.0.'):
    raise SystemExit(f'Expected Isaac Sim 6.0 family, got {version}')
urdf = root / 'x650_mount/urdf/x650_sprayer.urdf'
mesh = root / 'x650_mount/meshes/x650_frame.stl'
for path in (urdf, mesh):
    if not path.is_file() or not path.stat().st_size:
        raise SystemExit(f'X650 model asset missing: {path}')

from isaacsim import SimulationApp
app = SimulationApp({'headless': True})
try:
    import omni.kit.app
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    from pxr import Usd

    manager = omni.kit.app.get_app().get_extension_manager()
    for name in ('isaacsim.asset.importer.urdf', 'isaacsim.robot.schema',
                 'omni.scene.optimizer.core'):
        manager.set_extension_enabled_immediate(name, True)
    app.update()
    output_dir = root / 'assets/generated/isaac_6_0_x650'
    output_dir.mkdir(parents=True, exist_ok=True)
    config = URDFImporterConfig(
        urdf_path=str(urdf), usd_path=str(output_dir), robot_type='Aerial',
        merge_fixed_joints=False, merge_mesh=False, fix_base=False,
        collision_from_visuals=False, allow_self_collision=False,
    )
    usd_path = Path(URDFImporter(config).import_urdf())
    stage = Usd.Stage.Open(str(usd_path)) if usd_path.is_file() else None
    if not stage or not stage.GetDefaultPrim():
        raise RuntimeError(f'X650 USD import invalid: {usd_path}')
    names = {prim.GetName() for prim in stage.Traverse()}
    required = {'base_link','battery_6s','rail_mount','sprayer_body',
                'nozzle_head','trigger_crank','spray_tip'}
    if not required <= names or 'landing_risers' in names:
        raise RuntimeError(f'X650 USD link mismatch; missing={sorted(required-names)}')
    manifest = {
        'version': version,
        'urdf_path': str(urdf.relative_to(root)),
        'usd_path': str(usd_path.relative_to(root)),
        'default_prim': stage.GetDefaultPrim().GetPath().pathString,
        'frame_mesh': str(mesh.relative_to(root)),
        'frame_mesh_sha256': hashlib.sha256(mesh.read_bytes()).hexdigest(),
        'source_step': 'assets/source/x650-frame-2025-0825.stp',
        'import_settings': {'robot_type':'Aerial','merge_fixed_joints':False,
                            'fix_base':False,'collision_from_visuals':False},
    }
    output = root / 'assets/generated/x650_asset_manifest.json'
    output.write_text(json.dumps(manifest,indent=2)+'\n')
    print(f'Imported X650 USD: {usd_path}', flush=True)
finally:
    app.close()
