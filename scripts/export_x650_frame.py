"""Tessellate the assembled Holybro X650 STEP frame for Isaac URDF import.

Run from the project root with FreeCADCmd scripts/export_x650_frame.py.
The STEP uses Y up, Z lateral; the output uses Z up, Y lateral.
"""
from pathlib import Path

import FreeCAD as App
import Import
import MeshPart

root = Path.cwd()
source = root / 'assets/source/x650-frame-2025-0825.stp'
destination = root / 'x650_mount/meshes/x650_frame.stl'
destination.parent.mkdir(parents=True, exist_ok=True)

doc = App.newDocument('X650FrameExport')
Import.insert(str(source), doc.Name)
doc.recompute()
matches = [obj for obj in doc.Objects
           if obj.Label.startswith('X650-FRAME-2024CHU_ASM_73_1_ASM_ASM')
           and hasattr(obj, 'Shape') and not obj.Shape.isNull()]
if len(matches) != 1:
    raise RuntimeError(f'Expected one X650 frame assembly, found {len(matches)}')
shape = matches[0].Shape.copy()
box = shape.BoundBox
if not (500 < box.XLength < 520 and 480 < box.YLength < 500 and
        500 < box.ZLength < 520):
    raise RuntimeError(f'Unexpected source frame bounds: {box}')

# Rotate the STEP's vertical Y to Isaac's vertical Z; align mounting rail
# centerlines with the retained sprayer bridge in the existing URDF frame.
shape.rotate(App.Vector(0, 0, 0), App.Vector(1, 0, 0), 90)
shape.translate(App.Vector(0, 0, 29.809569))
mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=1.5,
                              AngularDeflection=0.55, Relative=False)
mesh.write(str(destination))
print(f'X650 frame mesh: {destination}; facets={mesh.CountFacets}; '
      f'bounds={mesh.BoundBox}', flush=True)
