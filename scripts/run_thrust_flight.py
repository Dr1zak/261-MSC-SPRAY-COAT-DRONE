#!/usr/bin/env python3
"""Apply four simulated rotor forces to the selected sprayer airframe."""
import argparse
import csv
import json
import math
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--workspace', type=Path, required=True)
parser.add_argument('--mode', choices=('hover', 'route'), default='hover')
parser.add_argument('--airframe', choices=('x650', 'x500', 'x650_study'), default='x650')
parser.add_argument('--duration', type=float, default=None)
parser.add_argument('--headless', action='store_true')
parser.add_argument('--keep-open', action='store_true',
                    help='Keep the Isaac GUI open after the flight until its window is closed')
parser.add_argument('--capture', type=Path, default=None,
                    help='Save an Isaac viewport image during physical flight')
parser.add_argument('--capture-at', type=float, default=8.0)
parser.add_argument('--capture-times', type=float, nargs='+', default=None,
                    help='Capture several viewport frames in one run (requires --capture)')
args, _ = parser.parse_known_args()
if args.duration is None:
    args.duration = 8.0 if args.mode == 'hover' else 110.0
root = args.workspace.resolve()
scene = root / ('assets/generated/x650_site_construction_site.usd'
                if args.airframe == 'x650' else
                'assets/generated/x500_site_construction_site.usd')
if not scene.is_file():
    raise SystemExit('Build the construction site first')
if args.duration <= 0:
    raise SystemExit('--duration must be positive')
if args.capture_times and not args.capture:
    raise SystemExit('--capture-times requires --capture')
if args.keep_open and args.headless:
    raise SystemExit('--keep-open requires the Isaac GUI')

from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless,
                     'renderer': 'MinimalRendering' if args.headless and not args.capture else 'RealTimePathTracing',
                     'disable_viewport_updates': args.headless and not args.capture})
try:
    import numpy as np
    import omni.usd
    import yaml
    import isaacsim.core.experimental.utils.app as app_utils
    from isaacsim.core.experimental.prims import RigidPrim
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    from construction_site import WAYPOINTS, OBSTACLES
    from flight_dynamics import (FlightController, ThrustCurve, added_mass_properties,
                                 rotation_from_wxyz, urdf_mass_kg)
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    config_path = ('config/x650_flight.yaml' if args.airframe == 'x650' else
                   'config/x650_study.yaml' if args.airframe == 'x650_study' else
                   'config/flight_controller.yaml')
    config = yaml.safe_load((root / config_path).read_text())
    components = ([] if args.airframe == 'x650' else
                  config['mass_components'] if args.airframe == 'x650_study' else
                  yaml.safe_load((root / 'config/airframe_estimate.yaml').read_text())['components'])
    curve = ThrustCurve(root / config['thrust_curve_csv'], config['max_continuous_throttle'])
    added_mass, added_com, _ = (added_mass_properties(components) if components else
                                (0.0, np.zeros(3), np.zeros((3,3))))
    model_urdf = root / config.get('model_urdf', 'x500_mount_v6/urdf/x500_v2_sprayer.urdf')
    mass = urdf_mass_kg(model_urdf)
    if not omni.usd.get_context().open_stage(str(scene)):
        raise RuntimeError(f'Could not open {scene}')
    for _ in range(10):
        app.update()
    stage = omni.usd.get_context().get_stage()
    base_path = config.get('robot_base_path', '/World/X500/Geometry/base_link')
    if args.airframe == 'x650_study':
        from x650_proxy import author_proxy
        author_proxy(stage, base_path, config)
    base_mat = UsdGeom.Xformable(stage.GetPrimAtPath(base_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    center_sum = Gf.Vec3d(0, 0, 0)
    imported_mass = 0.0
    riser_prim = None
    riser_mass = 0.0
    riser_world_com = None
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(base_path) or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        api = UsdPhysics.MassAPI(prim)
        link_mass = float(api.GetMassAttr().Get())
        local_com = Gf.Vec3d(*api.GetCenterOfMassAttr().Get())
        world_com = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).Transform(local_com)
        center_sum += world_com * link_mass
        imported_mass += link_mass
        if args.airframe == 'x650_study' and prim.GetName() == 'landing_risers':
            riser_prim, riser_mass, riser_world_com = prim, link_mass, world_com
    if abs(imported_mass - mass) > .01:
        raise RuntimeError(f'Imported mass {imported_mass:.3f} kg differs from URDF {mass:.3f} kg')
    if args.airframe == 'x650_study':
        if riser_prim is None:
            raise RuntimeError('X500 landing-risers link not found for X650 mass correction')
        # X650 already has skids. Omit the X500-only risers from the study.
        for child in Usd.PrimRange(riser_prim):
            if child.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(child).GetCollisionEnabledAttr().Set(False)
        UsdGeom.Imageable(riser_prim).MakeInvisible()
        UsdPhysics.MassAPI(riser_prim).GetMassAttr().Set(1e-6)
        center_sum -= riser_world_com * riser_mass
        imported_mass -= riser_mass
        mass -= riser_mass
    imported_com = np.array(base_mat.GetInverse().Transform(center_sum / imported_mass))
    base_api = UsdPhysics.MassAPI(stage.GetPrimAtPath(base_path))
    old_base_mass = float(base_api.GetMassAttr().Get())
    old_base_com = np.array(base_api.GetCenterOfMassAttr().Get(), dtype=float)
    old_inertia = np.diag(np.array(base_api.GetDiagonalInertiaAttr().Get(), dtype=float))
    new_base_mass = old_base_mass + added_mass
    new_base_com = (old_base_mass * old_base_com + added_mass * added_com) / new_base_mass
    def shift_inertia(amount, offset):
        return amount * (np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset))
    new_inertia = old_inertia + shift_inertia(old_base_mass, old_base_com-new_base_com)
    for item in components:
        new_inertia += shift_inertia(float(item['mass_kg']),
                                     np.asarray(item['position_m'])-new_base_com)
    base_api.GetMassAttr().Set(new_base_mass)
    base_api.GetCenterOfMassAttr().Set(Gf.Vec3f(*map(float, new_base_com)))
    base_api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*map(float, np.diag(new_inertia))))
    if args.airframe == 'x650_study':
        stage.GetRootLayer().Export(str(root / 'assets/generated/x650_study_site.usd'))
    mass += added_mass
    center_of_mass = (imported_mass * imported_com + added_mass * added_com) / mass
    controller = FlightController(config, mass, center_of_mass, curve)
    print(f'Flight model: mass={mass:.3f} kg, COM in base frame={tuple(center_of_mass)} m; '
          f'hover throttle={controller.motors[0]:.3f}', flush=True)
    if not args.headless and args.mode == 'hover':
        set_camera_view(eye=(12, -9, 10), target=(9, 8, 1), camera_prim_path='/OmniverseKit_Persp')
    dt = 1 / 60
    SimulationManager.set_physics_dt(dt)
    body = RigidPrim(base_path)
    app_utils.play()
    app.update()
    physics_start_time = SimulationManager.get_simulation_time()

    def read_state():
        positions, orientations = body.get_world_poses()
        velocities, angular = body.get_velocities()
        return (positions.numpy()[0].astype(float), orientations.numpy()[0].astype(float),
                velocities.numpy()[0].astype(float), angular.numpy()[0].astype(float))

    log = []
    capture_schedule = []
    capture_futures = []
    if args.capture:
        capture_base = args.capture if args.capture.is_absolute() else root / args.capture
        times = sorted(set(args.capture_times or [args.capture_at]))
        if any(t < 0 or t >= args.duration for t in times):
            raise ValueError('Capture times must be nonnegative and before flight duration')
        for when in times:
            path = (capture_base.with_name(f'{capture_base.stem}_t{int(when):03d}s{capture_base.suffix}')
                    if args.capture_times else capture_base)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.unlink(missing_ok=True)
            capture_schedule.append((when, path))
    min_structure_clearance = float('inf')
    min_boundary_clearance = float('inf')
    route_index = 1
    target = np.array([2, 2, 1.5], dtype=float)
    for step in range(math.ceil(args.duration / dt)):
        position, quat, velocity, angular = read_state()
        if (not args.headless or args.capture) and step % 12 == 0:
            camera_offset = ((-1.5, -2.0, 1.3) if args.mode == 'hover' else
                             (-4.0, -6.0, 4.0))
            set_camera_view(eye=tuple(position + camera_offset),
                            target=tuple(position), camera_prim_path='/OmniverseKit_Persp')
        if args.mode == 'route':
            # Sequential target following, with a lift before the first travel leg.
            if step * dt > 2.0:
                target = np.asarray(WAYPOINTS[route_index], dtype=float)
                if np.linalg.norm(position - target) < config['waypoint_tolerance_m'] and route_index < len(WAYPOINTS)-1:
                    route_index += 1
        rotor_forces = controller.command(
            position, quat, velocity, angular, target, dt,
            max_horizontal_speed=config['route_speed_m_s'] if args.mode == 'route' else None)
        for force, location, spin, throttle in zip(rotor_forces, controller.rotor_positions,
                                                    controller.spin, controller.motors):
            body.apply_forces_and_torques_at_pos(
                forces=[[0.0, 0.0, float(force)]],
                torques=[[0.0, 0.0, float(spin * curve.reaction_torque(throttle))]],
                positions=[location.tolist()], local_frame=True)
        app.update()
        if not args.headless and step % 600 == 0:
            print(f'GUI flight step {step}: simulated elapsed '
                  f'{SimulationManager.get_simulation_time()-physics_start_time:.3f}s; '
                  f'expected {step*dt:.3f}s', flush=True)
        if capture_schedule and step * dt >= capture_schedule[0][0]:
            viewport = get_active_viewport()
            if viewport is None:
                raise RuntimeError('Isaac viewport unavailable for flight capture')
            viewport.resolution = (1280, 720)
            _, capture_path = capture_schedule.pop(0)
            capture_futures.append((capture_path,
                                    capture_viewport_to_file(viewport, str(capture_path))))
        if step % 6 == 0:
            tilt = math.degrees(math.acos(float(np.clip(rotation_from_wxyz(quat)[2, 2], -1, 1))))
            log.append([step * dt, *position, *velocity, tilt, *rotor_forces,
                        *controller.motors, sum(curve.current_a(controller.motors)), route_index])
        if not np.all(np.isfinite(position)) or position[2] < -0.2 or position[2] > 15:
            raise RuntimeError(f'Flight became unstable at command t={step*dt:.2f}s, '
                               f'physics t={SimulationManager.get_simulation_time()-physics_start_time:.2f}s: '
                               f'{position}')
        if args.mode == 'hover' and np.linalg.norm(position[:2] - [2, 2]) > 5:
            raise RuntimeError(f'Hover drift exceeded 5 m at t={step*dt:.2f}s: {position}')
        if args.mode == 'route':
            x, y = position[:2]
            min_boundary_clearance = min(min_boundary_clearance, x, 50-x, y, 50-y)
            for a,b,c,d in OBSTACLES:
                min_structure_clearance = min(min_structure_clearance,
                    math.hypot(max(a-x, 0, x-b), max(c-y, 0, y-d)))
            if not (0.5 < x < 49.5 and 0.5 < y < 49.5):
                raise RuntimeError(f'Drone left the construction-site boundary: {position}')
            footprint_margin = (.7 if args.airframe.startswith('x650') else .4)
            if any(a-footprint_margin < x < b+footprint_margin and
                   c-footprint_margin < y < d+footprint_margin for a,b,c,d in OBSTACLES):
                raise RuntimeError(f'Drone entered a construction structure footprint: {position}')

    final_pos, final_q, final_vel, final_omega = read_state()
    if capture_futures:
        for _ in range(120):
            if all(path.is_file() and path.stat().st_size > 0
                   for path, _ in capture_futures):
                break
            app.update()
        for path, _ in capture_futures:
            if not path.is_file() or path.stat().st_size == 0:
                raise RuntimeError(f'Flight viewport capture did not finish: {path}')
    final_tilt = math.degrees(math.acos(float(np.clip(rotation_from_wxyz(final_q)[2, 2], -1, 1))))
    if args.mode == 'hover':
        success = (abs(final_pos[2]-1.5) < .1 and
                   np.linalg.norm(final_pos[:2]-[2,2]) < .2 and
                   np.linalg.norm(final_vel) < .2 and final_tilt < 5)
    else:
        success = (route_index == len(WAYPOINTS)-1 and
                   np.linalg.norm(final_pos - WAYPOINTS[-1]) < .75 and
                   np.linalg.norm(final_vel) < .3 and final_tilt < 5 and
                   min_structure_clearance > (.7 if args.airframe.startswith('x650') else .4) and
                   min_boundary_clearance > (.7 if args.airframe.startswith('x650') else .5))
    if args.keep_open:
        app_utils.pause()
    else:
        app_utils.stop()
    report_dir = root / 'reports'
    report_dir.mkdir(exist_ok=True)
    report_tag = ('x650_' if args.airframe == 'x650' else
                  'x650_study_' if args.airframe == 'x650_study' else '')
    csv_path = report_dir / f'thrust_{report_tag}{args.mode}_telemetry.csv'
    with csv_path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['time_s','x_m','y_m','z_m','vx_m_s','vy_m_s','vz_m_s',
                         'tilt_deg','rotor_1_n','rotor_2_n','rotor_3_n','rotor_4_n',
                         'motor_1_throttle','motor_2_throttle','motor_3_throttle','motor_4_throttle',
                         'estimated_motor_current_a','waypoint'])
        writer.writerows(log)
    design_current_limit = float(config.get('design_current_limit_a', 30.0))
    design_envelope_pass = bool(
        mass <= 4*curve.force(.70)/config['gravity_m_s2'] and
        max(row[16] for row in log) <= design_current_limit)
    summary = {'mode': args.mode, 'airframe': args.airframe,
               'duration_s': args.duration, 'modeled_mass_kg': mass,
               'added_hardware_mass_kg': added_mass,
               'excluded_x500_riser_mass_kg': riser_mass,
               'bench_curve': config['thrust_curve_csv'],
               'model_urdf': str(model_urdf.relative_to(root)),
               'scene_usd': str(scene.relative_to(root)),
               'bench_voltage_v': float(config.get('bench_voltage_v', 16.0)),
               'max_continuous_throttle': curve.max_throttle,
               'hover_throttle_estimate': float(curve.throttle_for_force(mass*config['gravity_m_s2']/4)),
               'max_observed_throttle': max(max(row[12:16]) for row in log),
               'max_estimated_motor_current_a': max(row[16] for row in log),
               'mean_estimated_motor_current_a': float(np.mean([row[16] for row in log])),
               'samples_over_pm02_wire_30a': int(sum(row[16] > 30 for row in log)),
               'design_current_limit_a': design_current_limit,
               'design_envelope_pass': design_envelope_pass,
               'seventy_percent_throttle_mass_limit_kg':
                   float(4*curve.force(.70)/config['gravity_m_s2']),
               'hardware_envelope_pass': bool(
                   design_envelope_pass and config.get('power_path_verified', True)),
               'center_of_mass_base_m': list(center_of_mass),
               'success': bool(success),
               'final_position_m': final_pos.tolist(), 'final_velocity_m_s': final_vel.tolist(),
               'final_angular_velocity_rad_s': final_omega.tolist(),
               'max_altitude_m': max(row[3] for row in log),
               'max_tilt_deg': max(row[7] for row in log),
               'waypoint_index': route_index, 'telemetry': str(csv_path.relative_to(root))}
    if capture_futures:
        summary['viewport_captures'] = [str(path.relative_to(root)) for path, _ in capture_futures]
    if args.mode == 'route':
        summary['min_structure_clearance_m'] = min_structure_clearance
        summary['min_boundary_clearance_m'] = min_boundary_clearance
    (report_dir / f'thrust_{report_tag}{args.mode}_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary), flush=True)
    if not success:
        raise RuntimeError(f'{args.mode} flight did not meet final position and stability criteria')
    if args.keep_open:
        print('Flight complete; Isaac Sim GUI remains open until you close its window.', flush=True)
        while app.is_running():
            app.update()
except BaseException:
    import traceback
    traceback.print_exc()
    raise
finally:
    app.close()
