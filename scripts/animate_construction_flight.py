#!/usr/bin/env python3
"""Author and play a collision-checked kinematic site preview in Isaac Sim."""
import argparse
import math
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--workspace', type=Path, required=True)
parser.add_argument('--airframe', choices=('x650','x500'), default='x650')
parser.add_argument('--headless', action='store_true')
parser.add_argument('--seconds', type=float, default=None,
                    help='Playback duration; 0 only writes the animated USD')
args, _ = parser.parse_known_args()
root = args.workspace.resolve()
source = root / f'assets/generated/{args.airframe}_site_construction_site.usd'
output = root / f'assets/generated/{args.airframe}_construction_flight.usd'
if not source.is_file():
    raise SystemExit('Build the site first: ./scripts/run_isaac_scene.sh --scenario construction_site --headless --frames 0 --no-ros-clock')

from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless,
                     'renderer': 'MinimalRendering' if args.headless else 'RealTimePathTracing',
                     'disable_viewport_updates': args.headless})

try:
    print('Isaac flight: app initialized', flush=True)
    import omni.timeline
    import omni.usd
    from isaacsim.core.utils.viewports import set_camera_view
    from pxr import Usd, UsdGeom
    from construction_site import WAYPOINTS, validate_flight_path

    validate_flight_path()
    stage = Usd.Stage.Open(str(source))
    if not stage:
        raise RuntimeError(f'Cannot open site USD: {source}')
    robot_path = '/World/' + args.airframe.upper()
    robot = stage.GetPrimAtPath(robot_path)
    if not robot:
        raise RuntimeError(f'Site USD does not contain {robot_path}')
    physics = robot.GetVariantSets().GetVariantSet('Physics')
    print(f'Isaac flight: robot={robot.GetPath()} variants={robot.GetVariantSets().GetNames()}', flush=True)
    if not physics or not physics.SetVariantSelection('none'):
        raise RuntimeError('Could not disable imported robot physics for kinematic preview')

    speed = 5.0  # metres per second; visualization only
    fps = 60.0
    dt = 0.1
    legs = [math.dist(a, b) for a, b in zip(WAYPOINTS, WAYPOINTS[1:])]
    total_time = sum(legs) / speed
    translate_ops = [op for op in UsdGeom.Xformable(robot).GetOrderedXformOps()
                     if op.GetOpName() == 'xformOp:translate']
    if len(translate_ops) != 1:
        raise RuntimeError('Expected one robot root translation op')
    translate = translate_ops[0]
    cumulative = [0.0]
    for distance in legs:
        cumulative.append(cumulative[-1] + distance / speed)
    sample_count = math.ceil(total_time / dt)
    for i in range(sample_count + 1):
        t = min(i * dt, total_time)
        leg = next((j for j in range(len(legs)) if t <= cumulative[j+1]), len(legs)-1)
        f = (t - cumulative[leg]) / (cumulative[leg+1] - cumulative[leg])
        a, b = WAYPOINTS[leg], WAYPOINTS[leg+1]
        position = tuple(a[k] + (b[k] - a[k]) * f for k in range(3))
        if not translate.Set(position, Usd.TimeCode(t * fps)):
            raise RuntimeError('Could not author robot translation')
    stage.SetTimeCodesPerSecond(fps)
    stage.SetFramesPerSecond(fps)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(total_time * fps)
    stage.GetRootLayer().Export(str(output))
    print('Isaac flight: animation exported', flush=True)
    print(f'Isaac flight: translate samples={translate.GetNumTimeSamples()}', flush=True)

    start = UsdGeom.Xformable(robot).ComputeLocalToWorldTransform(Usd.TimeCode(0)).ExtractTranslation()
    midway = UsdGeom.Xformable(robot).ComputeLocalToWorldTransform(Usd.TimeCode(total_time * fps / 2)).ExtractTranslation()
    print(f'Isaac flight: position check start={tuple(start)} midpoint={tuple(midway)}', flush=True)
    if math.dist(start, midway) < 5:
        raise RuntimeError('Robot animation has no substantial flight motion')
    print(f'Wrote {output}; {len(WAYPOINTS)} waypoints; {total_time:.1f} s; '
          f'start={tuple(start)} midpoint={tuple(midway)}', flush=True)

    playback = total_time if args.seconds is None else args.seconds
    if playback < 0:
        raise ValueError('--seconds must be nonnegative')
    if playback:
        if not omni.usd.get_context().open_stage(str(output)):
            raise RuntimeError('Isaac could not open the animated flight USD')
        for _ in range(15):
            app.update()
        if not args.headless:
            set_camera_view(eye=(68, -30, 52), target=(25, 25, 0),
                            camera_prim_path='/OmniverseKit_Persp')
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_looping(False)
        timeline.set_current_time(0)
        timeline.play()
        target_frames = math.ceil(min(playback, total_time) * fps)
        if playback >= total_time:
            target_frames += 3  # let Kit reach the authored final time sample
        for _ in range(target_frames):
            if not app.is_running():
                break
            app.update()
        observed = timeline.get_current_time()
        timeline.stop()
        if observed + 1 / fps < min(playback, total_time):
            raise RuntimeError(f'Isaac timeline stopped early at {observed:.2f} s')
        print(f'Isaac timeline advanced to {observed:.2f} s; '
              f'animation has {total_time:.1f} s of kinematic flight', flush=True)
except BaseException:
    import traceback
    traceback.print_exc()
    raise
finally:
    app.close()
