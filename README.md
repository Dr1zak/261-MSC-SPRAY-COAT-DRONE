# Construction-site marking simulation workspace

This workspace contains an X650 CAD-frame Isaac model carrying the existing sprayer, the original X500 assets, ROS 2 interface definitions, a deterministic **algorithm test backend**, a construction site, and a prototype force-driven flight controller. It is not yet an autonomous marking demonstration. `run_demo.sh` fails clearly in Isaac mode until the flight controller, bridge command/state nodes, and ROS mission execution are integrated.

## Environment

Target: Ubuntu 24.04, Python 3.12, ROS 2 Jazzy. The installed Isaac Sim version is `6.0.1-rc.7+release.42383.32955d8d.gl`. Its URDF importer and a short site preview ran successfully here. Use the matching 6.0 family [URDF importer](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/importer_exporter/import_urdf.html), [ROS 2 bridge](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/ros2_tutorials/ros2_landing_page.html), and [requirements](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/requirements.html) as references for further integration. NVIDIA lists an RTX 4080 with 16 GB VRAM and Linux driver 580.95.05 as its 6.0 x86 minimum. This host has an RTX 4070 Laptop GPU with 8 GB and driver 580.178.04; this narrow import/preview succeeded, but larger scenes and full autonomous operation remain unvalidated.

```bash
sudo apt install python3-yaml python3-pytest python3-colcon-common-extensions python3-rosdep
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
./scripts/check_environment.sh
./scripts/prepare_assets.sh  # validates 90 named STEP part mappings and 18 URDF visual meshes
./scripts/build.sh
./scripts/run_demo.sh --scenario baseline --backend test
./scripts/run_experiments.sh --scenarios baseline obstacles noise failures --seeds 1 2 3
PYTHONPATH=src/construction_autonomy pytest -q tests/unit
```

The following asset/site commands were executed successfully on this host with Isaac Sim 6.0.1 RC, NVIDIA driver 580.178.04, and an RTX 4070 Laptop GPU (8 GB):

```bash
export ISAAC_SIM_ROOT="$HOME/isaacsim"  # default if omitted
./scripts/convert_model.sh                # X650 by default; --airframe x500 for the original
./scripts/run_isaac_scene.sh --scenario baseline --headless --frames 60
# For an interactive preview, omit --headless. That mode has not been tested here.
```

`convert_model.sh` invokes Isaac's own `python.sh`, imports the dedicated X650 URDF with the STEP-derived frame visual and simplified collision proxies, preserves fixed links including `spray_tip`, and writes USD under `assets/generated/`. The baseline scene script adds the 50 × 50 m site, four controls, 50 targets, obstacles where configured, a ground plane, the robot, and one `/clock` graph. The scene command itself does not command flight. The ROS bridge remains to be connected to the flight controller and mission executor.

The default `./scripts/run_demo.sh --scenario baseline` requests Isaac and reports the outstanding integration instead of substituting test results. The test backend uses idealized travel at configured speed and point-local deposition. It does not prove physics, localization, stabilization, ROS service/action behavior, or safe flight. Reports appear in unique `reports/<scenario>_<method>_<seed>_<hash>_<serial>/` directories, with `report.json` and `targets.csv`; experiment summaries are in `reports/experiment_summary.*`.

## Construction-site flight preview

Run the local construction environment and a 32-second X650 flight tour in Isaac Sim:

```bash
./scripts/run_construction_flight.sh
# For an automated, windowless playback check:
./scripts/run_construction_flight.sh --headless --seconds 3
```

The script creates `assets/generated/x650_site_construction_site.usd` and `assets/generated/x650_construction_flight.usd`. The 50 × 50 m scene has a concrete building frame, scaffold, container, pallets, excavation, tower crane, roads, landing pad, and perimeter barriers. Structures have collision geometry. The scripted route uses 13 waypoints and is checked against conservatively expanded structure footprints and site boundaries. It takes off from and returns to the landing pad. The separate `construction_site` scenario contains eight unobstructed ground targets and uses the same obstacle map for the deterministic test backend. Pass `--airframe x500` to preview the original aircraft.

This flight is **kinematic animation** of the imported X650; the robot's physics variant is disabled in the animated USD. It demonstrates scene layout, scale, visibility, and a clear visual flight corridor. It is not a thrust-controlled or autonomous flight simulation, and it does not execute the marking mission. Use the X650 force-driven command below for a physics flight. See [construction-site details](docs/construction_site.md).

## Force-driven Isaac flight

```bash
./scripts/run_thrust_flight.sh --mode hover       # 8-second takeoff and hover
./scripts/run_thrust_flight.sh --mode route       # 110-second construction-site tour
./scripts/run_thrust_flight.sh --mode route --headless
```

This command builds the construction scene and applies four simulated rotor forces to the imported floating-base articulation each physics step. The controller reads Isaac pose and velocity, controls position and attitude, allocates thrust using the four rotor locations and the imported assembly center of mass, models motor response, and checks site bounds and structure clearances. The headless full route returned to `(2.00, 2.00, 0.80)` m with a minimum 0.995 m structure clearance; commanded horizontal speed is limited to 2 m/s. Results are in `reports/thrust_route_summary.json` and `reports/thrust_route_telemetry.csv`.

The supplied URDF's 2.744 kg omits flight hardware. The simulator now adds an itemized 1.025 kg estimate and uses a published 16 V motor/propeller bench curve, with a 90% continuous throttle cap and an 8% collective control reserve. The simulated route can track the path, but its mass and estimated current exceed the chosen 70% throttle and PM02 lead limits. `success` checks simulated tracking; `hardware_envelope_pass` checks these hardware limits. This is not a physical flight validation. See the [mass and propulsion calibration](docs/propulsion_calibration.md) and [construction-site details](docs/construction_site.md).

For the **X650 CAD-frame aircraft**, run `./scripts/run_x650_flight.sh --mode hover --headless` or `--mode route --headless`. It imports `x650_mount/urdf/x650_sprayer.urdf`, uses a mesh tessellated from Holybro's X650 STEP, retains the sprayer and rail mount, and omits the X500-only risers. Its URDF includes a nominal 2.000 kg X650 kit and 0.750 kg 6S battery; the controller uses the published MN4014 bench curve as a propeller proxy. The native model passed the 8 s hover and 110 s route checks. Results use `reports/thrust_x650_*`; the site scene is `assets/generated/x650_site_construction_site.usd`. Run `./scripts/convert_x650_model.sh` to rebuild the imported aircraft. To regenerate its frame mesh from the official STEP, use `scripts/export_x650_frame.py` with FreeCADCmd as described in [X650 model integration](docs/x650_model_integration.md). For geometry and hardware limitations, see the [X650 upgrade study](docs/x650_upgrade_study.md).

To inspect the static X650 site in Isaac's GUI, run `~/isaacsim/python.sh scripts/open_x650_isaac_gui.py --workspace "$PWD"`. To watch force-driven flight in the GUI, omit `--headless` from the X650 flight command.

Original model assets stay under `x500_mount_v6/` and the root STEP file. `config/asset_mapping.yaml` records 90 named payload/attachment parts mapped from the supplied assembly metadata. Re-run `prepare_assets.sh` to refresh hashes. The URDF visual meshes are scaled from millimeters to meters; they are pre-existing tessellations whose group membership is checked against the STEP product names. The remaining aircraft parts in the 336-product root STEP are not individually mapped. `assets/generated/asset_manifest.json` records provenance. See `docs/model_integration.md`.

The CSV format is `point_id,x_m,y_m,z_m,mark_type,priority`; `site` frame, meters, ground elevation zero, `dot` mark, positive integer priority. Priority 1 executes before 2; route optimization is within each group. The baseline has 50 deterministic points in a 50 × 50 m site. The planner assumes obstacles are vertical columns across all flight altitudes. Conventional 140–275 minute and UAV 38–70 minute figures are unvalidated proposal estimates, not measured productivity.

## Troubleshooting and limits

- `prepare_assets.sh` exit 2 means a source path, mesh, unit declaration, or named STEP mapping failed validation. It does not attempt USD import.
- If `nvidia-smi` fails in an isolated shell, recheck from the integrated terminal; sandbox permissions can hide the working driver.
- If ROS packages fail to build, source Jazzy first and run `rosdep install` as above.
- `run_demo.sh --backend test` is an algorithm check only. There is currently no RViz configuration or live robot visualization.
- Noise, dropout and navigation-timeout YAML fields are specified for future physics integration. The test backend rejects scenarios with those fields and records them as failed experiment runs. It currently applies only deposition error and seeded deposition failures.
