# X500 mass and propulsion calibration (2026-10-11)

The supplied URDF assigns 0.610 kg to `base_link`, the X500 frame-kit estimate, and 2.74439 kg to all six inertial links. The visual motor shapes are present, but motor, ESC, propeller, battery, and flight electronics masses were omitted from that inertial budget. `config/airframe_estimate.yaml` adds 1.0251 kg at estimated base-frame locations. The force simulation now uses **3.76949 kg** and a recomputed aggregate center of mass; it does not modify the source URDF.

| Omitted item | Added mass | Basis |
|---|---:|---|
| 4S 5000 mAh LiPo | 448 g | [Ovonic 4S 5000 mAh XT60 product](https://www.ovonicshop.com/products/ovonic-14-8v-5000mah-4s1p-50c-lipo-battery-with-xt60-trx-plug-for-rc-car-trucks), ±20 g published; representative pack, not the actual battery |
| Four motors | 256 g | [T-Motor AIR2216II KV920](https://store.tmotor.com/product/air-gear-450II-combo-set-no-esc.html), 64±2 g each; a 2216 KV920 proxy for Holybro's listed motor |
| Four ESCs | 84 g | Same T-Motor page, 21 g each; proxy for Holybro's BLHeli S 20 A ESC |
| Four flying propellers | 52 g | Same T-Motor page, T1045 13 g each; Holybro supplies six, but only four fly |
| Pixhawk 6C, plastic case | 34.6 g | [Holybro specification](https://holybro.com/products/pixhawk-6c) |
| M10 GPS | 32 g | [Holybro specification](https://holybro.com/collections/gps/products/m10-gps) |
| PM02 V3 power module | 20 g | [Holybro specification](https://holybro.com/products/pm02-v3-12s-power-module) |
| Airborne SiK radio | 23.5 g | [Holybro radio specification](https://holybro.com/products/sik-telemetry-radio-v3) |
| PDB; wiring/fasteners | 25 g; 50 g | Explicit allowances, unmeasured |

[Holybro's X500 V2 listing](https://holybro.com/products/x500-v2-kits) identifies the 610 g frame, four 2216 KV920 motors, four 20 A ESCs, 1045 props, PDB, and 4S 3000–5000 mAh battery recommendation. Its 1.5 kg payload at 70% throttle is a product claim, not a measured result for this custom assembly. The source model's payload/mount mass is about 2.134 kg. The battery location, PDB/wire allowance, actual hardware variants, and all payload inertias require weighing and CG measurement.

`config/air2216ii_t1045_16v.csv` transcribes the [manufacturer's 16 V bench table](https://store.tmotor.com/product/air-gear-450II-combo-set-no-esc.html) for AIR2216II KV920 with a T1045 propeller. It provides 30–100% throttle points at 5% intervals: thrust, propeller reaction torque, current, RPM, and power. The simulation uses monotonic linear interpolation, including a synthetic zero point, inverse interpolation from requested force to throttle, first-order motor lag, and interpolated torque/current. The 90% continuous throttle cap gives 1191 g or 11.68 N per rotor, just below the propeller's listed 1.2 kg thrust limitation. The 100% point is a bench/peak reference, not a continuous flight allowance. Motor lag remains an unmeasured 0.06 s assumption.

This is a **published proxy curve, not a bench measurement of this aircraft**. Its motor, ESC, exact prop variant, 16 V supply, cooling, and installation can differ from the X500 hardware. Battery voltage sag, ground effect, prop interference, wind, and motor heating are not represented. Record a calibrated thrust-stand sweep for each installed motor/ESC/prop at representative battery voltages before treating the curve as a hardware prediction.

For a direct curve, secure each installed motor/ESC/prop combination to a guarded thrust stand; zero a calibrated load cell before each run. At stabilized battery voltages spanning charged to near-end-of-flight conditions, record thrust, voltage at the ESC, current, RPM, and winding/case temperature at 5% throttle steps through the intended operating range. Repeat each point three times in ascending and descending order after temperature stabilizes, and retain raw readings and uncertainty. Derive reaction torque from a torque fixture or calibrated power/RPM measurement, then replace the proxy CSV with the measured curve. Weigh the assembled aircraft and measure its CG with the actual battery and filled/empty payload states; update `airframe_estimate.yaml` accordingly.

The model needs roughly **77.1% average throttle to hover**. Four rotors at the published 70% point lift 3.256 kg, leaving this estimate **0.513 kg above** that mass. The manufacturer's [PM02 V3 page](https://holybro.com/products/pm02-v3-12s-power-module) rates its pre-soldered XT60 lead at 30 A continuous; the simulated steady motor draw is about 39 A before avionics. The current estimate assumes 16 V and does not model sag or temperature. `hardware_envelope_pass` therefore fails even when the simulated path reaches its target. Reducing trajectory speed cannot resolve steady hover current; the aircraft needs a weighed mass budget and a properly rated power path/propulsion choice before physical flight.
