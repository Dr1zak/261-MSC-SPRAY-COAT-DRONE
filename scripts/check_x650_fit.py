#!/usr/bin/env python3
"""Report dimensional clues from the imported X650 CAD and existing X500 mount.

This is a coarse rail/ground check, not a solid-interference or strength check.
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
inspection = json.loads((ROOT / 'assets/generated/x650_cad_inspection.json').read_text())
shapes = inspection['shapes']
rails = [s for s in shapes if s['label'].startswith('CARBON-FIBER-TUBE320_23_1')]
skids = [s for s in shapes if s['label'].startswith('CARBON-FIBER-TUBE-320_3_1')]
if len(rails) != 2 or len(skids) != 2:
    raise SystemExit('Expected two central mount rails and two skid tubes in X650 STEP')

# Holybro STEP: X longitudinal, Y vertical, Z lateral. Existing X500 model:
# X longitudinal, Y lateral, Z vertical. Align the central rail centerlines.
rail_z_cad = sum(s['center_mm'][1] for s in rails) / 2
x500_rail_z = 16.809569
vertical_offset = rail_z_cad - x500_rail_z
rail_spacing = abs(rails[0]['center_mm'][2] - rails[1]['center_mm'][2])
x500_spacing = 30.275911 - (-29.724089)
rail_min_x = max(s['bounds_mm'][0] for s in rails)
rail_max_x = min(s['bounds_mm'][1] for s in rails)
skid_bottom = min(s['bounds_mm'][2] for s in skids)
sprayer_bottom = -254 + vertical_offset
old_riser_bottom = -279 + vertical_offset
report = {
    'source': inspection['source'],
    'status': 'dimensional screening only; no exact interference, stress, or attachment validation',
    'x650_central_rail_center_spacing_mm': rail_spacing,
    'x500_bridge_rail_center_spacing_mm': x500_spacing,
    'nominal_spacing_difference_mm': rail_spacing - x500_spacing,
    'x650_central_rail_overlap_x_mm': [rail_min_x, rail_max_x],
    'x500_bridge_clamp_x_mm': [-74, 60],
    'clamp_positions_on_rails': rail_min_x < -74 < rail_max_x and rail_min_x < 60 < rail_max_x,
    'x650_skid_tube_bottom_cad_y_mm': skid_bottom,
    'sprayer_bottom_in_x650_cad_y_mm': sprayer_bottom,
    'nominal_sprayer_ground_clearance_mm': sprayer_bottom - skid_bottom,
    'x500_optional_riser_bottom_in_x650_cad_y_mm': old_riser_bottom,
    'x500_optional_riser_vs_x650_skid_bottom_mm': old_riser_bottom - skid_bottom,
}
destination = ROOT / 'reports/x650_dimensional_fit.json'
destination.parent.mkdir(exist_ok=True)
destination.write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
