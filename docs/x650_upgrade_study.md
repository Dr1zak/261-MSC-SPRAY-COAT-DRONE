# X650 propulsion upgrade study

This document records the earlier X650 dynamics study, which used an X500 import with an X650 visual overlay. The current default simulation uses the [dedicated X650 CAD-frame model](x650_model_integration.md): run `./scripts/run_x650_flight.sh --mode hover --headless` or `--mode route --headless`. For the original X500 force-driven profile, use `./scripts/run_thrust_flight.sh`. Neither simulation establishes a completed physical conversion.

## Replacement bill of materials and modeling basis

| Item | Study input | Source and status |
|---|---:|---|
| Holybro X650 full kit, without battery | 2.000 kg | [Holybro X650 listing](https://holybro.com/products/x650-development-kit); includes frame, four MN4014 KV330 motors, four Tekko32 F4 45 A ESCs, Gemfan 1555 props, PDB, Pixhawk 6C, GPS and radio. Kit mass is a published nominal total. |
| 6S 5200 mAh battery | 0.750 kg | Same Holybro listing's battery/payload example. Specific pack model, size, current rating and actual weight have not been selected or verified. |
| Existing sprayer and rail mount, excluding X500 skid risers | 1.88216 kg | Supplied `x500_mount_v6` URDF estimates minus 0.25223 kg of X500-only risers. Bridge dimensions have been screened against X650 rails; attachment strength is unverified. |
| **Estimated takeoff mass** | **4.63216 kg** | Nominal sum; no installed aircraft has been weighed. |

The X650 full-kit mass replaces the X500's modeled 0.610 kg frame and its separate flight hardware estimate. In the earlier `config/x650_study.yaml` proxy, an X500 base link was a dynamics container with 0.590 kg added centrally and 0.200 kg at each new rotor position. The 0.200 kg pod value is an inertia distribution estimate (T-Motor lists 171 g for each MN4014 including cables), not a part-level weighing. A 0.750 kg battery was added centrally and the X500 risers hidden. The current X650 URDF embeds that 2.000 kg kit distribution and a separate 0.750 kg battery link, and excludes the X500 risers entirely.

The older visual overlay in `assets/generated/x650_study_site.usd` hides the X500 frame mesh and draws 650 mm-class arms, motor pods and 15.5-inch propeller disks. Its underlying X500 frame collision proxies remain. The current `assets/generated/x650_site_construction_site.usd` instead references the dedicated X650 URDF and STEP-derived frame visual with X650-specific collision proxies. Both retain the supplied payload/mount geometry. Neither is an X650 assembly or landing-gear fit signoff. The site route check uses a 0.7 m horizontal structure margin.

### X650 CAD screening

The [official X650 STEP download](https://docs.holybro.com/drone-development-kit/x650-development-kit/download) is stored at `assets/source/x650-frame-2025-0825.stp` (SHA-256 `62cf031a85c59a7f023c2b525115497dfb0821de9c964c1a5cf905c549d156f7`). FreeCAD imported 497 solid-bearing objects; `assets/generated/x650_cad_inspection.json` records their bounding boxes. Run `python3 scripts/check_x650_fit.py` to reproduce `reports/x650_dimensional_fit.json`.

In the source STEP coordinate system, X is longitudinal, Y is vertical, and Z is lateral. The two central 320 mm mounting tubes are 60 mm apart, with overlapping X range −155 to +165 mm. The X500 bridge has 60 mm rail spacing and clamps at X=−74/+60 mm, so those first-order dimensions align. The nominal sprayer bottom would be about **24.2 mm above the X650 skid-tube bottom** after rail centerline alignment. The old optional X500 risers would sit about 0.8 mm below that plane and must not be treated as X650 landing gear. These are bounding-box and centerline checks only: no assembled-solid intersection, tolerance, clamp torque, rail crush, battery volume, hose sweep, or landing-load check has passed.

## Propulsion and electrical model

`config/mn4014_kv330_p15_22v2.csv` transcribes [T-Motor's MN4014 KV330 + P15*5CF 22.2 V bench data](https://store.tmotor.com/product/mn4014-kv330-motor-navigator-type.html): thrust/current/RPM/power at 50, 65, 75, 85 and 100% throttle. The X650 kit instead ships **Gemfan 1555** props, so this is a proxy curve for a similar diameter, not the exact installed system. The manufacturer table gives no reaction torque. The CSV's torque column is explicitly a `0.012 m × thrust` assumption inherited from the prior model; it must be replaced by measured torque. The 0.06 s motor time constant is also assumed. `max_continuous_throttle` is set to 85% in the study, leaving nominal thrust reserve.

At 4.63216 kg, equal-load hover requires about 1.158 kg per rotor, 65.3% throttle and approximately 23.8 A motor-only at the 22.2 V bench condition. The proxy curve's 70% collective lift is 5.160 kg, about 0.528 kg above the nominal study mass. [Holybro advertises](https://holybro.com/products/x650-development-kit) a 6.3 kg takeoff limit at 70% with its *different* installed propeller; that number is not substituted into the T-Motor curve. These are two different evidence bases and must not be blended.

The X650 [PDB is rated 60 A continuous](https://holybro.com/products/power-distribution-board-pdb). The Pixhawk 6C-compatible [PM02 V3 supplied XT60 lead is rated 30 A continuous](https://holybro.com/products/pm02-v3-12s-power-module), despite the module PCB's higher rating. `design_current_limit_a: 60` describes a **proposed redesigned power path**, not the existing lead. `power_path_verified: false` forces `hardware_envelope_pass: false` until a selected pack, connectors, wire, protection, PDB and module are installed and qualified. The report's `design_envelope_pass` checks only the nominal 70% thrust and proposed 60 A limit. Voltage sag, heating, 6S battery capacity, and avionics draw remain outside the simulation.

## Latest Isaac checks

The corrected 4.63216 kg study passed the 8 s hover and 110 s construction-site route checks in headless Isaac Sim. Hover peak estimated motor current was 29.3 A; route peak was 33.7 A. The route's minimum center-to-structure clearance was 0.995 m, above the 0.7 m study margin. Both reports show `design_envelope_pass: true` and `hardware_envelope_pass: false`. See `reports/thrust_x650_study_hover_summary.json`, `reports/thrust_x650_study_route_summary.json`, and their telemetry CSVs. A subsequent route run with NVIDIA device access captured real Isaac viewport frames at 15, 30, 45 and 60 seconds in `reports/x650_route_frames/isaac_gpu_route_t*.png`. GPU access is unavailable inside the default workspace sandbox, so native viewport capture needs Isaac to run with device access.

## Remaining installation and validation work

1. Obtain the actual 6S battery dimensions. Place the sprayer solids in the X650 assembly, perform exact interference and tolerance checks, and redesign the bridge or clamps wherever needed. Qualify the X650 skid attachment and landing loads. Check 15.5-inch swept disks and cable routing throughout flex and landing deflection.
2. Select and install a power path with continuous and peak ratings covering measured mission current, including the PM02 lead or a compatible alternative; verify Pixhawk power-module interface and protection. The X500 4S ESCs and battery are not part of this design.
3. Weigh the assembled aircraft with full and empty cans and battery; measure CG and inertias or estimate them from verified CAD. Replace the study mass distribution and the X500 payload attachment geometry.
4. Bench-test each installed MN4014/Gemfan/ESC combination at representative 6S voltages. Record thrust, reaction torque, current, RPM and temperature. Replace the proxy curve, torque assumption and motor lag estimate.
5. Replace the current estimated inertias, sparse collision proxies, and unverified payload attachment with the finished X650 assembly. Retune control and rerun hover and route with battery sag and full/empty payload, then conduct restrained and controlled physical tests.

The current code does not command or configure real ESCs or firmware. It cannot purchase, fit, wire, or test hardware remotely.
