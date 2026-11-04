# Isaac asset and site-preview validation

Executed 2026-10-10 on Ubuntu 24.04 with Isaac Sim `6.0.1-rc.7+release.42383.32955d8d.gl`, NVIDIA driver `580.178.04`, and an RTX 4070 Laptop GPU with 8188 MB reported memory. This GPU is below NVIDIA's published 6.0 minimum, but the following narrow checks passed here.

| Command | Observed result |
|---|---|
| `./scripts/prepare_assets.sh` | 90 STEP product names mapped; 18 URDF visual meshes resolved; source hashes recorded |
| `./scripts/convert_model.sh` | Isaac URDF importer produced `assets/generated/isaac_6_0_x500_v6/x500_v2_sprayer/x500_v2_sprayer.usda`; opened stage had a default prim and expected base/marker link names |
| `./scripts/run_isaac_scene.sh --scenario baseline --headless --frames 60` | Saved `assets/generated/x500_site_baseline.usd`, read 50 targets, started ROS 2 bridge, and advanced simulation time 1.000 s |
| `./scripts/run_isaac_scene.sh --scenario obstacles --headless --frames 1 --no-ros-clock` | Saved `assets/generated/x500_site_obstacles.usd` with scenario obstacle colliders; advanced simulation time 0.017 s |
| `PYTHONPATH=src/construction_autonomy pytest -q tests/unit` | 8 passed |

The first scene startup spent roughly 104 seconds compiling GPU shaders. A later startup completed within about 10 seconds. The scene has a ground plane, visual site boundary, four survey controls, targets, optional obstacle collision boxes, referenced X500 USD, and a `/clock` action graph. The script does not command flight: no thrust actuator, navigation, localization adapter, mark service, mission action, or RViz view is connected. Creating the `/clock` graph and starting the bridge does not by itself prove external ROS topic delivery. The imported mass properties are prototype estimates and omit parts of the complete aircraft. STEP product names were audited against the supplied mesh groups; the import used existing STL tessellations, not a fresh STEP tessellation.

## Construction-site preview, 2026-10-11

`./scripts/run_construction_flight.sh --headless --seconds 3` built the procedural construction site and advanced the animated USD to 3.00 s. A GUI launch with `--seconds 1` advanced to 1.00 s. Full headless playback reached 32.05 s of a 32.1 s route, and the route sampling check found no structure or boundary intrusion. The animated USD contains 322 robot translation samples; its sampled midpoint is about `(43.44, 39.68, 5.0)` m, distinct from home `(2, 2, 0.8)` m. An Isaac viewport capture shows the constructed site and imported X500 at 3 s.

One full-run attempt logged Vulkan `ERROR_DEVICE_LOST` during startup while still advancing the timeline. An immediate retry had no GPU error and completed the route. The local GPU remains below NVIDIA's published 6.0 minimum. This preview uses robot kinematic animation, with its imported physics variant disabled; it does not validate rotor dynamics or collision response of the aircraft.

## Force-driven flight, 2026-10-11

The imported X500 has six rigid bodies with 2.744 kg of URDF mass. Its imported aggregate center of mass is `(0.01548, 0.00279, -0.10927)` m in the base frame. Four simulated rotor forces, reaction yaw torques, a motor lag, and position/attitude feedback were applied to the floating-base articulation in the construction scene. An 8 s takeoff/hover ended at `(2.004, 2.001, 1.500)` m with very low velocity and a maximum 2.23° tilt. A 110 s headless site tour ended at `(2.003, 1.999, 0.800)` m; minimum structure clearance was 0.995 m, minimum boundary clearance 1.999 m, maximum tilt 14.64°, and sampled maximum horizontal speed 2.01 m/s. The route run reported `success: true`; no footprint intrusion occurred. An offscreen Isaac capture of the physical hover was produced.

These are simulated dynamics results under assumed per-rotor thrust and motor response values. The incomplete aircraft mass model and missing measured thrust curve prevent any claim that the real X500/payload combination can fly this mission. The run does not invoke ROS mission actions or mark targets.

## Mass-adjusted flight and propulsion proxy, 2026-10-11

The model now adds 1.0251 kg of estimated battery, motors, ESCs, propellers, avionics, and wiring to the 2.74439 kg URDF import, updates base-link inertia, and recomputes aggregate COM. A published AIR2216II KV920/T1045 16 V bench table drives motor force, reaction torque, and current interpolation. The controller caps throttle at 90% and collective acceleration at 92% of capped thrust to retain attitude authority. These are modeling choices and a published proxy, not measurements of the assembled X500. Inputs and sources are in [propulsion calibration](propulsion_calibration.md).

With this model, the 8 s hover ended at `(2.00255, 2.00043, 1.49995)` m; peak tilt was 2.38° and mean estimated motor draw was 39.27 A. The 110 s route ended at `(2.00313, 2.00043, 0.80000)` m; minimum structure clearance was 0.993 m, peak tilt 16.75°, peak estimated motor draw 48.79 A, and mean 39.38 A. Both simulated trajectory checks passed. Before the collective reserve, the same mass/curve route had 24.78° peak tilt and 52.46 A peak estimated draw; the reserve reduced both peaks. The updated unit suite passed 10 tests.

The 70% throttle thrust corresponds to 3.256 kg total mass, 0.513 kg below the modeled aircraft. The PM02 lead is rated 30 A continuous, while 1094 of 1100 route telemetry samples exceeded 30 A in estimated *motor-only* draw. Both runs report `hardware_envelope_pass: false`. No real X500 bench thrust, complete-aircraft weighing, battery sag, heat, wind, endurance, or hardware power-path test has been performed. `success: true` denotes only simulated path tracking.
