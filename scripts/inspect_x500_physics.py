#!/usr/bin/env python3
"""Report imported rigid bodies and inertias needed by the flight controller."""
import argparse
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('--scene', type=Path, required=True)
args, _ = p.parse_known_args()

from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'renderer': 'MinimalRendering', 'disable_viewport_updates': True})
try:
    from pxr import Usd, UsdPhysics, UsdGeom
    stage = Usd.Stage.Open(str(args.scene.resolve()))
    if not stage:
        raise RuntimeError('Could not open scene')
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith('/World/X500'):
            continue
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            print('articulation', prim.GetPath(), flush=True)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            mass = UsdPhysics.MassAPI(prim)
            print('rigid', prim.GetPath(),
                  'mass', mass.GetMassAttr().Get() if mass else None,
                  'com', mass.GetCenterOfMassAttr().Get() if mass else None,
                  'inertia', mass.GetDiagonalInertiaAttr().Get() if mass else None,
                  'world', tuple(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()),
                  flush=True)
finally:
    app.close()
