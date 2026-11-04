#!/usr/bin/env python3
"""Isaac 6.0 site with the selected imported airframe."""
import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--workspace', type=Path, required=True)
parser.add_argument('--scenario', default='baseline')
parser.add_argument('--airframe', choices=('x650','x500'), default='x650')
parser.add_argument('--headless', action='store_true')
parser.add_argument('--frames', type=int, default=600)
parser.add_argument('--no-ros-clock', action='store_true')
args, _ = parser.parse_known_args()
root = args.workspace.resolve()
if args.airframe == 'x650':
    manifest_path = root / 'assets/generated/x650_asset_manifest.json'
    if not manifest_path.is_file():
        raise SystemExit('Convert the X650 URDF first: ./scripts/convert_x650_model.sh')
    record = json.loads(manifest_path.read_text())
else:
    manifest_path = root / 'assets/generated/asset_manifest.json'
    record = json.loads(manifest_path.read_text()).get('isaac_import')
    if record is None:
        raise SystemExit('Convert the X500 URDF first: ./scripts/convert_model.sh --airframe x500')
usd_path = root / record['usd_path']
if not usd_path.is_file():
    raise SystemExit(f'Converted robot USD missing: {usd_path}')
if args.frames < 0:
    raise SystemExit('--frames must be nonnegative')

from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless,
                     'renderer': 'MinimalRendering' if args.headless else 'RealTimePathTracing',
                     'disable_viewport_updates': args.headless})

try:
    print('Isaac scene: app initialized', flush=True)
    import csv
    import yaml
    import omni.usd
    import isaacsim.core.experimental.utils.app as app_utils
    import isaacsim.core.experimental.utils.stage as stage_utils
    from isaacsim.core.experimental.objects import DistantLight, GroundPlane
    from isaacsim.core.experimental.prims import XformPrim
    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import Gf, UsdGeom, UsdPhysics

    scenario_path = root / 'config/scenarios' / f'{args.scenario}.yaml'
    config = yaml.safe_load(scenario_path.read_text())
    if not args.no_ros_clock:
        import omni.graph.core as og
        app_utils.enable_extension('isaacsim.ros2.bridge')
        app.update()
    print('Isaac scene: ROS extension configured', flush=True)

    stage_utils.create_new_stage()
    print('Isaac scene: stage created', flush=True)
    stage_utils.set_stage_units(meters_per_unit=1.0)
    GroundPlane('/World/GroundPlane', positions=[0, 0, 0])
    light = DistantLight('/World/DistantLight')
    light.set_intensities(300)
    robot_path = '/World/' + args.airframe.upper()
    stage_utils.add_reference_to_stage(usd_path=str(usd_path), path=robot_path)
    print('Isaac scene: robot USD referenced', flush=True)
    XformPrim(robot_path, reset_xform_op_properties=True).set_world_poses(
        positions=[config['home'][0], config['home'][1],
                   0.431 if config.get('environment') == 'construction_site' else 0.281])
    stage = omni.usd.get_context().get_stage()

    def sphere(name, x, y, radius, color):
        prim = UsdGeom.Sphere.Define(stage, name)
        prim.CreateRadiusAttr(radius)
        prim.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdGeom.XformCommonAPI(prim).SetTranslate((float(x), float(y), radius))

    def box(name, x, y, z, sx, sy, sz, color, collision=False):
        prim = UsdGeom.Cube.Define(stage, name)
        prim.CreateSizeAttr(1.0)
        prim.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        xf = UsdGeom.XformCommonAPI(prim)
        xf.SetTranslate((x, y, z))
        xf.SetScale((sx, sy, sz))
        if collision:
            UsdPhysics.CollisionAPI.Apply(prim.GetPrim())

    xmin, xmax, ymin, ymax = map(float, config['bounds'])
    for name, x, y, sx, sy in (
        ('South', (xmin+xmax)/2, ymin, xmax-xmin, .08),
        ('North', (xmin+xmax)/2, ymax, xmax-xmin, .08),
        ('West', xmin, (ymin+ymax)/2, .08, ymax-ymin),
        ('East', xmax, (ymin+ymax)/2, .08, ymax-ymin),
    ):
        box('/World/Site/Boundary/'+name, x, y, .025, sx, sy, .05, (.2,.4,.8))
    for i,(x,y) in enumerate(((xmin,ymin),(xmin,ymax),(xmax,ymin),(xmax,ymax))):
        sphere(f'/World/Site/Control/Corner_{i}',x,y,.17,(.1,.8,.2))
    layout = root / config['layout']
    with layout.open(newline='') as stream:
        targets = list(csv.DictReader(stream))
    for row in targets:
        sphere('/World/Site/Targets/'+row['point_id'],float(row['x_m']),float(row['y_m']),.09,(.9,.6,.05))
    if config.get('environment') == 'construction_site':
        light.set_intensities(2500)
        from construction_site import OBSTACLES, build, validate_flight_path
        if tuple(tuple(map(float, item)) for item in config.get('obstacles', [])) != OBSTACLES:
            raise ValueError('Construction scenario obstacle footprints differ from its USD structures')
        validate_flight_path(tuple(map(float, config['bounds'])))
        build(stage, box)
    else:
        sphere('/World/Site/Home',*config['home'],.2,(.1,.5,.95))
        for index,(xa,xb,ya,yb) in enumerate(config.get('obstacles',[])):
            box(f'/World/Site/Obstacles/Obstacle_{index}',(xa+xb)/2,(ya+yb)/2,1.5,
                xb-xa,yb-ya,3.0,(.8,.15,.15),collision=True)

    if not args.no_ros_clock:
        og.Controller.edit(
            {'graph_path':'/World/ROSClock','evaluator_name':'execution'},
            {og.Controller.Keys.CREATE_NODES:[
                ('Tick','omni.graph.action.OnPlaybackTick'),
                ('SimTime','isaacsim.core.nodes.IsaacReadSimulationTime'),
                ('Clock','isaacsim.ros2.bridge.ROS2PublishClock')],
             og.Controller.Keys.CONNECT:[
                ('Tick.outputs:tick','Clock.inputs:execIn'),
                ('SimTime.outputs:simulationTime','Clock.inputs:timeStamp')],
             og.Controller.Keys.SET_VALUES:[('Clock.inputs:topicName','/clock')]})

    output = root / 'assets/generated' / f'{args.airframe}_site_{args.scenario}.usd'
    stage.GetRootLayer().Export(str(output))
    print(f'Saved Isaac site scene: {output}; targets={len(targets)}; robot={usd_path}', flush=True)
    print('Physics preview only: no thrust controller or autonomous mission is connected.')
    SimulationManager.set_physics_dt(1.0/60.0)
    app_utils.play()
    app.update()
    start_time = SimulationManager.get_simulation_time()
    for _ in range(args.frames):
        if not app.is_running():
            break
        app.update()
    end_time = SimulationManager.get_simulation_time()
    if args.frames and end_time <= start_time:
        raise RuntimeError('Isaac physics time did not advance')
    print(f'Isaac physics time advanced {end_time-start_time:.3f} s over requested {args.frames} frames', flush=True)
    app_utils.stop()
except BaseException:
    import traceback
    traceback.print_exc()
    raise
finally:
    app.close()
