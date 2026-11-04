# X650 Isaac model integration

The default Isaac conversion and site scene use `x650_mount/urdf/x650_sprayer.urdf`. Its frame visual comes from the official Holybro STEP file in `assets/source/x650-frame-2025-0825.stp`; its sprayer, nozzle linkage, and rail-mounted bridge come from the supplied X500 payload assembly. The X500 frame and X500-specific landing risers are absent from the X650 URDF. The four rotor locations, X650 frame collision proxies, battery inertial, and kit mass are authored for this model.

## Rebuild and run

The checked-in `x650_mount/meshes/x650_frame.stl` is the tessellated STEP assembly. To recreate it on this host, run the FreeCAD command below from the project root, then rebuild the URDF and Isaac asset:

```bash
XDG_DATA_HOME=/tmp/x650-freecad-home \
XDG_CONFIG_HOME=/tmp/x650-freecad-config \
XDG_CACHE_HOME=/tmp/x650-freecad-cache \
LD_LIBRARY_PATH=/snap/freecad/current/usr/lib:/snap/kf6-core24/current/usr/lib/x86_64-linux-gnu:/snap/freecad/current/usr/lib/x86_64-linux-gnu \
/snap/freecad/current/usr/bin/FreeCADCmd scripts/export_x650_frame.py
./scripts/convert_x650_model.sh
./scripts/run_x650_flight.sh --mode hover --headless
./scripts/run_x650_flight.sh --mode route --headless
```

`./scripts/convert_model.sh` also selects X650 by default. `--airframe x500` keeps the older conversion available. `./scripts/run_isaac_scene.sh --airframe x650 --scenario construction_site --headless --frames 0` creates the X650 site independently of the flight command. The output robot is `/World/X650`; the route and hover controller apply rotor forces to `/World/X650/Geometry/base_link`.

## Geometry and dynamics

The STEP uses X longitudinal, Y vertical, Z lateral coordinates. `export_x650_frame.py` rotates the full frame assembly +90° about X into Isaac's Z-up coordinates, then shifts it +29.809569 mm vertically so the source central mounting rail centerlines align with the retained bridge. The mesh is a visual asset with 0.001 URDF scale; the URDF uses sparse frame, arm, rail, motor, skid, and leg collision shapes for physics. Four translucent disks show the nominal 15.5-inch swept propeller areas. These disks are visual clearance guides, not colliders.

`scripts/build_x650_model.py` writes the URDF and `x650_mount/model_metadata.json`, including source and frame-mesh hashes. The URDF mass budget is 2.000 kg for the nominal X650 kit, 0.750 kg for a provisional 6S battery, and 1.88216 kg for the retained payload and bridge: **4.63216 kg total**. The 2 kg kit's inertial distribution and battery dimensions/placement are estimates; component measurements have not been supplied. Rotor thrust uses the [MN4014 bench-data proxy and controller assumptions](x650_upgrade_study.md), not an installed Gemfan 1555 thrust measurement.

The existing bridge's 60 mm rail spacing and clamp X positions pass a first-order dimension check against the X650 source CAD. The nominal sprayer-to-skid clearance is about 24.2 mm. Exact assembled-solid interference, clamp strength, cable and hose routing, propeller sweep under deflection, landing loads, and the real battery fit still require physical design verification. This model is for simulation, not a released hardware assembly.

## Current Isaac checks

The dedicated X650 model passed an 8 s force-driven hover and 110 s construction-site route in Isaac Sim 6.0.1 RC. The hover ended at approximately `(2.003, 2.000, 1.500)` m. The route returned to approximately `(2.003, 2.000, 0.800)` m, with minimum mapped structure clearance 0.994 m. The controller estimated peak motor-only currents of 28.7 A in hover and 33.8 A on the route. See `reports/thrust_x650_hover_summary.json`, `reports/thrust_x650_route_summary.json`, and the paired telemetry CSVs. Both checks passed simulated flight criteria; `hardware_envelope_pass` is false because the installed power path and exact propeller thrust have not been verified.

An NVIDIA-rendered frame of the native model in flight is saved at `reports/x650_native_hover.png`.
