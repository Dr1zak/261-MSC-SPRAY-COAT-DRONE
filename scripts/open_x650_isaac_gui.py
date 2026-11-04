#!/usr/bin/env python3
"""Keep the X650 CAD-frame scene open in the Isaac Sim GUI for inspection."""
import argparse
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--workspace', type=Path, required=True)
args, _ = parser.parse_known_args()
stage_path = args.workspace.resolve() / 'assets/generated/x650_site_construction_site.usd'
if not stage_path.is_file():
    raise SystemExit(f'X650 site stage missing: {stage_path}')

from isaacsim import SimulationApp
app = SimulationApp({'headless': False, 'renderer': 'RealTimePathTracing'})
try:
    import omni.usd
    from isaacsim.core.utils.viewports import set_camera_view

    if not omni.usd.get_context().open_stage(str(stage_path)):
        raise RuntimeError(f'Could not open {stage_path}')
    for _ in range(20):
        app.update()
    set_camera_view(eye=(10, -5, 8), target=(2, 2, 1.5),
                    camera_prim_path='/OmniverseKit_Persp')
    print(f'X650 scene open in Isaac Sim GUI: {stage_path}', flush=True)
    print('Close the Isaac Sim window to exit.', flush=True)
    while app.is_running():
        app.update()
finally:
    app.close()
