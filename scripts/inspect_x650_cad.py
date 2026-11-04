"""Extract mount and frame dimensions from the official Holybro X650 STEP assembly.

Run from the project root with FreeCADCmd scripts/inspect_x650_cad.py.
"""
import json
from pathlib import Path

import FreeCAD as App
import Import


root = Path.cwd()
source = root / 'assets/source/x650-frame-2025-0825.stp'
destination = root / 'assets/generated/x650_cad_inspection.json'
doc = App.newDocument('X650Inspection')
Import.insert(source, doc.Name)
doc.recompute()
items = []
for obj in doc.Objects:
    if not hasattr(obj, 'Shape') or obj.Shape.isNull():
        continue
    box = obj.Shape.BoundBox
    items.append({
        'label': obj.Label,
        'name': obj.Name,
        'bounds_mm': [box.XMin, box.XMax, box.YMin, box.YMax, box.ZMin, box.ZMax],
        'size_mm': [box.XLength, box.YLength, box.ZLength],
        'center_mm': [(box.XMin + box.XMax)/2, (box.YMin + box.YMax)/2,
                      (box.ZMin + box.ZMax)/2],
    })
with open(destination, 'w') as stream:
    json.dump({'source': source, 'shape_count': len(items), 'shapes': items}, stream, indent=2)
print('Imported shapes:', len(items))
