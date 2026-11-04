# Verification matrix

| Requirement | Implementation | Evidence | Status |
|---|---|---|---|
| CSV validation | `core.py:load_layout` | `tests/unit/test_core.py` | passed unit tests |
| Inflated collision-free path | `core.py:GridPlanner` | unit tests | passed unit tests |
| Home-inclusive routing | `core.py:route` | unit tests | passed unit tests |
| Mark idempotency and threshold | `core.py:MarkStore`, `verify` | unit tests | passed unit tests |
| Fifty-point algorithm run | deterministic backend | baseline report | passed test backend only |
| ROS interface generation | `construction_interfaces` | `./scripts/build.sh` | passed, 2 packages built |
| ROS mission action/services | interfaces only | no node tests | not implemented |
| Named STEP payload mapping | `config/asset_mapping.yaml`, `prepare_assets.py` | source/STEP name audit | passed: 90 names mapped, 18 meshes validated |
| URDF → USD asset import | `scripts/convert_model.sh` | Isaac 6.0.1 RC; expected frames present | passed |
| Isaac site scene preview | `scripts/run_isaac_scene.sh` | 50 targets; ROS bridge started; 60 frames = 1.000 s | passed for scene stepping |
| Isaac force-driven takeoff and hover | `run_thrust_flight.py`, `flight_dynamics.py` | 8 s hover: final altitude 1.500 m; max tilt 2.23° | passed for estimated model |
| Isaac construction-site flight | same controller and `construction_site.py` | 110 s tour: returned home, 0.995 m minimum structure clearance | passed for estimated model |
| ROS flight data path and marking mission | no command/state adapters or mission nodes | no system tests | not implemented |
| RViz, live TF, full mission | no runtime adapters | no system tests | not implemented |
