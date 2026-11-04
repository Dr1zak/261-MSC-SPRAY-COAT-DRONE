#!/usr/bin/env python3
"""Isaac Sim 6.0 standalone URDF→USD import of the supplied X500 V6 model."""
import argparse
import json
from pathlib import Path

P = argparse.ArgumentParser()
P.add_argument('--workspace', type=Path, required=True)
P.add_argument('--isaac-root', type=Path, required=True)
args, _ = P.parse_known_args()
root = args.workspace.resolve()
version = (args.isaac_root / 'VERSION').read_text().strip()
if not version.startswith('6.0.'):
    raise SystemExit(f'Expected Isaac Sim 6.0 family, got {version}')
manifest_path = root / 'assets/generated/asset_manifest.json'
manifest = json.loads(manifest_path.read_text())
if manifest['mapped_step_parts'] != 90:
    raise SystemExit('Run ./scripts/prepare_assets.sh before importing')

from isaacsim import SimulationApp
app = SimulationApp({'headless': True})

try:
    import omni.kit.app
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    from pxr import Usd

    manager = omni.kit.app.get_app().get_extension_manager()
    for name in ('isaacsim.asset.importer.urdf', 'isaacsim.robot.schema', 'omni.scene.optimizer.core'):
        manager.set_extension_enabled_immediate(name, True)
    app.update()
    urdf = root / 'x500_mount_v6/urdf/x500_v2_sprayer.urdf'
    output_dir = root / 'assets/generated/isaac_6_0_x500_v6'
    output_dir.mkdir(parents=True, exist_ok=True)
    config = URDFImporterConfig(
        urdf_path=str(urdf), usd_path=str(output_dir), robot_type='Aerial',
        merge_fixed_joints=False, merge_mesh=False, fix_base=False,
        collision_from_visuals=False, allow_self_collision=False,
    )
    usd_path = Path(URDFImporter(config).import_urdf())
    if not usd_path.is_file():
        raise RuntimeError(f'URDF importer returned missing USD: {usd_path}')
    stage = Usd.Stage.Open(str(usd_path))
    if not stage:
        raise RuntimeError(f'USD cannot be opened: {usd_path}')
    default_prim = stage.GetDefaultPrim()
    if not default_prim:
        raise RuntimeError('Imported USD lacks a default prim; cannot reference into site')
    prim_names = {p.GetName() for p in stage.Traverse()}
    missing = {'base_link', 'nozzle_head', 'trigger_crank', 'spray_tip'} - prim_names
    if missing:
        raise RuntimeError(f'Imported USD missing expected URDF links/frames: {sorted(missing)}')
    manifest['conversion_status'] = 'URDF visual STL and collision proxies imported to USD; Isaac site/flight not validated'
    manifest['isaac_import'] = {
        'version': version, 'usd_path': str(usd_path.relative_to(root)),
        'default_prim': default_prim.GetPath().pathString,
        'config': {'robot_type':'Aerial','merge_fixed_joints':False,'merge_mesh':False,
                   'fix_base':False,'collision_from_visuals':False,'allow_self_collision':False},
        'visual_source': 'supplied link-local STL meshes, checked against 90 named STEP payload parts',
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    print(f'Imported X500 USD: {usd_path}')
finally:
    app.close()
