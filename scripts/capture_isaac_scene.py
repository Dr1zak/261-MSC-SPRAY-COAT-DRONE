#!/usr/bin/env python3
"""Capture the generated X500 Isaac Sim site from the installed Isaac viewport."""
import argparse
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--scene', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--view', choices=('robot', 'site', 'construction', 'construction_drone'), default='robot')
parser.add_argument('--time', type=float, default=0.0)
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp

app = SimulationApp({
    'headless': True,
    'renderer': 'RealTimePathTracing',
    'disable_viewport_updates': False,
})

try:
    import omni.usd
    import omni.timeline
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    from pxr import Usd, UsdGeom, UsdLux

    if not omni.usd.get_context().open_stage(str(args.scene.resolve())):
        raise RuntimeError(f'Could not open {args.scene}')
    for _ in range(20):
        app.update()

    stage = omni.usd.get_context().get_stage()
    light = UsdLux.DomeLight.Define(stage, '/World/CaptureDomeLight')
    light.CreateIntensityAttr(1000.0)

    if args.view == 'robot':
        # The blue home marker occupies the robot's position in this preview.
        UsdGeom.Imageable(stage.GetPrimAtPath('/World/Site/Home')).MakeInvisible()
        eye, target = (3.2, 0.0, 1.65), (2.0, 2.0, 0.32)
    elif args.view == 'site':
        eye, target = (25.0, -5.0, 85.0), (25.0, 25.0, 0.0)
    elif args.view == 'construction':
        eye, target = (62.0, -20.0, 42.0), (25.0, 25.0, 0.0)
    else:
        robot = stage.GetPrimAtPath('/World/X500')
        current = UsdGeom.Xformable(robot).ComputeLocalToWorldTransform(
            Usd.TimeCode(args.time * stage.GetTimeCodesPerSecond())).ExtractTranslation()
        target = tuple(current)
        eye = (current[0] - 3.0, current[1] - 4.0, current[2] + 2.5)
    set_camera_view(eye=eye, target=target, camera_prim_path='/OmniverseKit_Persp')
    omni.timeline.get_timeline_interface().set_current_time(args.time)
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError('Isaac viewport unavailable')
    viewport.resolution = (1280, 720)
    for _ in range(100):
        app.update()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    capture = capture_viewport_to_file(viewport, str(args.output.resolve()))
    for _ in range(240):
        app.update()
        if capture.done():
            break
    if not capture.done():
        raise TimeoutError('Isaac viewport capture did not finish')
    capture.result()
    if not args.output.is_file():
        raise RuntimeError(f'No image written to {args.output}')
    print(f'Captured {args.output.resolve()} ({args.output.stat().st_size} bytes)', flush=True)
finally:
    app.close()
