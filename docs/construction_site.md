# Construction-site environment and flight tour

The site is authored locally from USD primitives by `scripts/construction_site.py`; it needs no external asset server. This keeps the scene small enough for this host's 8 GB RTX 4070 Laptop GPU. Isaac Sim also offers downloadable environment and prop libraries if a later visual upgrade is wanted: [Isaac Sim asset overview](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/assets/usd_assets_overview.html).

## Run

```bash
./scripts/run_construction_flight.sh
```

This command builds the construction site and opens a 32.1-second kinematic flight playback in Isaac Sim. Use `--headless --seconds 3` for a short automated check, or `--headless` for the whole route without a window. The site can be generated independently with:

```bash
./scripts/run_isaac_scene.sh --scenario construction_site --headless --frames 60 --no-ros-clock
```

Generated files:

| File | Purpose |
|---|---|
| `assets/generated/x650_site_construction_site.usd` | Static physics scene with X650 and collidable construction structures |
| `assets/generated/x650_construction_flight.usd` | Same scene with time-sampled X650 translation and physics disabled on the robot |
| `reports/x650_native_hover.png` | NVIDIA-rendered viewport of the force-driven X650 hover |

Pass `--airframe x500` to `run_construction_flight.sh` for the original X500 preview. Older screenshots named `assets/generated/isaac_construction_*.png` depict that legacy X500 scene.

The route starts at `(2, 2, 0.8)` m and tours the site at about 4–5 m altitude and 5 m/s before returning to the pad. A route check samples each segment every 0.1 m and reserves 0.85 m from boundaries and structure footprints. These footprints are conservative vertical columns, including the crane jib. The geometry and scenario map are checked for agreement when the site is built. The construction scenario's eight marking targets remain a test-backend layout; flight playback does not visit or mark them as a mission.

The flight USD is a **visual, kinematic preview**. For a separate X650 physics-driven run, use `./scripts/run_x650_flight.sh --mode hover` or `--mode route`. That script applies four forces and rotor reaction torques to the imported PhysX articulation at 60 Hz and records pose, speed, tilt, and rotor forces in `reports/`. It computes the imported assembly center of mass before allocating thrust. The route checks actual PhysX positions against conservative obstacle footprints every step and stops on intrusion.

The X650 URDF contains the nominal 2.000 kg kit, 0.750 kg battery, and retained sprayer, totaling 4.632 kg. The controller interpolates a published MN4014 bench curve as a proxy for the installed propellers and caps continuous throttle at 85%; response time remains assumed. Reports separate trajectory `success` from `hardware_envelope_pass`. The X650 passed the simulated hover and route, but its actual power path and propeller thrust remain unverified. It does not model aerodynamic drag, downwash, battery depletion, wind, or spray deposition. Hardware flight feasibility and autonomous ROS marking remain unvalidated. See the [X650 integration record](x650_model_integration.md). Isaac force application follows the [Isaac Sim rigid prim API](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/extensions/isaacsim.core.experimental.prims/docs/index.html).
