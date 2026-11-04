# X500 V2 with compact aerosol sprayer — V6 fit-check prototype

The supplied X500 STL and URDF are combined with the compact V5 sprayer, four rail clamps, an adapter bridge, and four proposed landing-riser shoes. One upright 65.7 mm can, the selected 2V025-style valve, two DS3240 servo references, and the electronics enclosure are retained. The nozzle rotates about a vertical axis while continuing to point down.

**The assembly is for dimensional review and joint simulation. It is not ready to fly.** The added payload estimate is approximately **2.13 kg**, including assumed full-can mass and solid-density structural parts. This exceeds Holybro's documented approximately 1 kg payload with a 4S 5000 mAh battery at 70% throttle. Actual component weights and the complete aircraft mass must be established before a lighter version can be designed. The supplied base inertia represents a 0.610 kg frame-kit estimate, not a measured complete aircraft.

## Open the result

- `urdf/x500_v2_sprayer.urdf`: combined model, with all meshes referenced relative to this file. Keep `meshes/` beside `urdf/`; copying the URDF alone is insufficient.
- `urdf/joint_positions.json`: released and pressed joint examples, in radians.
- `cad/X500_Sprayer_V6_FIT_CHECK.FCStd`: whole drone and attached mechanism in FreeCAD. The supplied drone remains a mesh; the added mechanism is solid CAD.
- `cad/Sprayer_with_X500_Mount_FIT_CHECK.step`: mechanism and mount solids, importable into Fusion 360. The dense drone mesh is not embedded in this STEP; use `meshes/x500_frame.stl` in the same coordinate system.
- `cad/X500_Rail_Mount_and_Risers_FIT_CHECK.step`: new attachment parts only.
- `stl_FIT_CHECK_ONLY/`: 17 individual new dry parts, in millimetres. These are fit-check exports, not pressure components or qualified flight hardware.
- `stl_V5_FIT_CHECK_ONLY/`: the 14 previously exported dry sprayer parts, included in the full CAD package.
- `X500_mounted_overview.png` and `X500_mount_clearance.png`: views rendered from CAD geometry.

No native `.f3d` file is claimed. STEP preserves solid geometry for Fusion import; the included Python code regenerates the design in FreeCAD.

## URDF joints and frames

| Joint | Parent → child | Motion |
|---|---|---|
| `rail_mount_fixed` | `base_link` → `rail_mount` | Fixed |
| `landing_risers_fixed` | `base_link` → `landing_risers` | Fixed |
| `sprayer_mount_fixed` | `rail_mount` → `sprayer_body` | Fixed translation `(0.040, 0.00027591, -0.262)` m |
| `nozzle_yaw` | `sprayer_body` → `nozzle_head` | Z rotation, `-1.570796327 … 0` rad |
| `can_trigger` | `sprayer_body` → `trigger_crank` | Y rotation, `-0.201357921 … 0` rad |
| `spray_tip_fixed` | `nozzle_head` → `spray_tip` | Fixed outlet reference |

At both nozzle orientations, `spray_tip` is at `(0.075, -0.01972409, -0.240)` m relative to `base_link`. Discharge is along local **−Z**. Negative yaw is clockwise when viewed from above. The 15 mm trigger crank moves its roller centre down 3 mm at the lower limit; actual can actuation travel/force must be measured. Can-stem contact and servo control are not simulated.

All URDF lengths are metres; STL vertices are millimetres with explicit `0.001` scale. The source drone `base_link` and original inertial values are retained. Its original large solid collision box was replaced by sparse plate, arm, motor, rail and landing-gear proxies so it does not falsely fill the space occupied by the payload. Payload collision proxies are similarly simplified. Adjacent-joint/internal-contact filtering may need simulator configuration.

The upstream flexible supply hose appears only in the nominal-pose CAD images. It is omitted from the articulated URDF visual so a rigid hose does not falsely detach from the moving inlet. Its estimated mass is retained. The short downstream hose rotates with the valve/nozzle assembly. This does not validate the supply hose's full sweep.

For a gravity/contact preview above a ground plane at world Z=0, place `base_link` initially around Z=0.281 m with the risers fitted. For a joint-only preview, hold the root fixed. A free root has no thrust model and will fall. There are no propeller joints, flight controller, ROS controller plugin, fluid particles or downwash model. Import into Isaac Sim/Gazebo has **not** been run here. The empty `spray_tip` link is only a frame; importers that collapse fixed joints may collapse it into its parent.

The `1 N·m` effort and `1 rad/s` speed limits are explicit simulation placeholders, not DS3240 specifications. Servo supply remains a nominal 6 V choice within the user's 4.8–6.8 V range. The valve coil voltage and current remain unconfirmed.

## Mechanical attachment

CAD uses the supplied frame coordinate system without centering or rescaling the drone. Measured rail centrelines are approximately Y=−29.724 and +30.276 mm, Z=16.810 mm. Their spacing is 60 mm. The mesh diameter is 9.4 mm; Holybro specifies nominal 10 mm rails, so the prototype bore is 10.2 mm. Measure the actual rods before machining.

Four 9 mm-long split clamps sit at X=−74 and +60 mm. The 156 × 104 × 6 mm bridge sits below the existing battery tray. Forward cap corners have reliefs for frame screw ends. The bridge connects to the V5 top plate through four M4 clearance holes at X=−37/+31, Y=−33.724/+34.276 mm, matching the 68 × 68 mm V5 slot pattern. Two extra holes clear the existing spine screw heads.

The tightest checked adapter clearance to the existing screw bounding envelopes is about 0.51 mm. Bridge-to-landing-leg surface clearance is about 9.64 mm. These nominal CAD gaps do not include manufacturing/assembly tolerances.

Clamps and bridge are intended as machined aluminium reference geometry. The small holes in the clamp bases are nominal 2.5 mm M3 tap-drill holes; threads are not modeled. Upper clamp screws and lower bridge screws are separate. The lower bridge holes have a nominal countersink. Exact screw lengths, thread engagement, tightening torque and carbon-tube crush limits require a hardware drawing and installation test; fasteners are not included in the mass estimate. Do not assume a plastic fit-check print can replace the aluminium load path.

The battery itself is absent from the supplied STL. The new bridge is only about 3.3 mm below the bottom of the modeled battery tray; a real battery or hanging straps could invalidate the fit. Battery installation, wiring, propeller clearance and aircraft balance remain installation checks.

## Landing clearance

The mechanism's lowest point is Z=−254 mm, while the original ground-contact plane is approximately Z=−185.467 mm. It therefore extends **68.5 mm below the stock landing gear**. The supplied stock gear cannot support a normal flat-ground landing with this module unchanged.

The four orange skid shoes propose a new ground plane at Z=−279 mm, 93.5 mm below the old one. This gives 25 mm under the module base and 39 mm under the nozzle. These tall shoes demonstrate clearance only: clamp friction, anti-rotation, buckling, impact, tipping and lateral stiffness have not been qualified. They are not an approved landing-gear design. Their stock-skid bores are 10.2 mm and also require actual tube measurements.

## Physics and mass limits

`mass_breakdown.csv` lists every estimate. Assumed purchased values include a 450 g full can, 72 g per servo, 200 g valve assembly and 80 g electronics. Structural density assumptions are aluminium 2.7 g/cm³ and polymer 1.25 g/cm³ at solid volume. The actual printed mass depends on material and process; reducing infill is not a substitute for load qualification. Can height and several purchased-part dimensions remain references.

Link centres of mass are CAD-volume centroids weighted by those estimates. Each part's local inertia is approximated by a uniform box having its bounding dimensions; tensors are combined with the parallel-axis theorem. The resulting positive-definite tensors are internally consistent estimates, not experimental identification. `mass_report.json` gives the current numbers. The modeled base-plus-payload total is about 2.74 kg, **excluding missing aircraft battery/avionics and omitted fasteners**, so it is not a takeoff-mass claim.

The estimated payload weight is about 20.9 N. Its centroid is roughly 20 mm forward of the supplied origin, giving about 0.42 N·m static pitch moment about that origin; at an illustrative 3g load these become 62.8 N and 1.25 N·m. These are load inputs, not stress or fatigue results. The whole-aircraft centre of gravity depends strongly on battery position and actual masses.

The hover mark remains a spray-development problem. In still air, a geometric fan width follows `W = 2h tan(theta/2)`; a 50 mm width at 100 mm outlet height corresponds to about 28° fan angle. This says nothing about uniform film thickness, the 25 mm minor dimension, droplet drift or rotor downwash. URDF does not predict any of those. The original 25 × 50 mm even-coverage objective needs controlled spray tests at actual height with the selected can, valve and commercial nozzle.

The pressure path remains commercial tubing/fittings/valve/nozzle reference geometry. A 4 mm stem OD is not a flow-orifice specification. Valve pressure rating alone does not establish paint/solvent compatibility, atomization or anti-drip performance. Opening the valve before pressing the can is a control sequence, not a way to eliminate its required pressure rating. No new pressure-bearing printed part is supplied.

## Verification and reproduction

`urdf_validation.json` records XML/tree checks, relative mesh paths, positive inertias, inertia triangle inequalities and sampled joint kinematics. `cad_validation.json` records closed new-part STLs, STEP round-trip checks, and attachment clearance checks against the source frame. The landing-leg sweep uses conservative 16 mm cylindrical proxies; the nominal hose-to-leg gap is approximately 7.3 mm. Full dynamic hose, structural and flight testing were not performed.

To regenerate: run `build_mount.py` with a Python runtime containing FreeCAD/Part/Import/MeshPart (tested with FreeCAD 0.21). Then run `make_urdf.py` and `validate_urdf.py` with Python3/NumPy, `validate_cad.py` with FreeCAD Python, and `render_assembly.py` with Python3/NumPy/Pillow. `source_v5/` contains the source CAD, palette/geometry and renderer needed to rebuild; `reference/` preserves the user's input files and geometry inspection data. Update `mass_for()` in `build_mount.py` with measured purchased-part masses and replace the base inertial values with a complete-aircraft estimate before dynamics work.

References accessed 2026-10-09: [Holybro X500 V2 documentation](https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2) specifies the rails and approximately 1 kg payload with the stated battery/throttle conditions. The [Holybro store specification](https://holybro.com/products/x500-v2-kits) uses a different payload basis, excluding the battery; do not interchange those figures. The user's exact STL and URDF determine this assembly's coordinates.
