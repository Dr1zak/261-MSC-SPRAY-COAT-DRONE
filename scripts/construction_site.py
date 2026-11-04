"""Local, procedural 50 m construction site and conservative flight corridor."""
from __future__ import annotations

import math

# Footprints include overhanging crane parts. The route treats these as columns.
OBSTACLES = (
    (17.5, 30.5, 16.5, 30.5),  # unfinished building
    (31.5, 38.5, 30.5, 39.5),  # scaffold
    (6.5, 15.5, 30.5, 35.5),   # container
    (29.0, 43.0, 10.0, 15.0),  # crane and jib
    (7.5, 14.5, 16.5, 24.5),   # excavation
    (7.5, 14.5, 7.5, 11.5),    # pallet stack
)

# A tour of the working area at safe altitude, with takeoff and landing at home.
WAYPOINTS = (
    (2.0, 2.0, 0.8),
    (5.0, 5.0, 4.0),
    (16.5, 6.0, 4.0),
    (16.5, 14.0, 5.0),
    (16.5, 38.0, 5.0),
    (25.0, 43.0, 5.0),
    (43.0, 43.0, 5.0),
    (45.0, 28.0, 5.0),
    (45.0, 18.0, 5.0),
    (45.0, 5.0, 5.0),
    (25.0, 5.0, 5.0),
    (5.0, 5.0, 4.0),
    (2.0, 2.0, 0.8),
)


def validate_flight_path(bounds=(0, 50, 0, 50), margin=0.85):
    """Check the swept XY route against bounds and every solid footprint."""
    for a, b in zip(WAYPOINTS, WAYPOINTS[1:]):
        steps = max(1, math.ceil(math.dist(a, b) / 0.1))
        for i in range(steps + 1):
            p = tuple(a[k] + (b[k] - a[k]) * i / steps for k in range(3))
            if not (bounds[0] + margin <= p[0] <= bounds[1] - margin and
                    bounds[2] + margin <= p[1] <= bounds[3] - margin and p[2] >= 0.8 - 1e-9):
                raise ValueError(f'flight path outside safe site volume: {p}')
            for xmin, xmax, ymin, ymax in OBSTACLES:
                if xmin - margin <= p[0] <= xmax + margin and ymin - margin <= p[1] <= ymax + margin:
                    raise ValueError(f'flight path enters a structure footprint: {p}')


def build(stage, box):
    """Author visible, collidable construction structures on an Isaac USD stage."""
    from pxr import Gf, UsdGeom, UsdPhysics

    sand = (.52, .42, .29)
    concrete = (.64, .66, .64)
    steel = (.23, .29, .34)
    orange = (.95, .38, .04)
    yellow = (.95, .72, .08)

    def cube(name, x, y, z, sx, sy, sz, color, solid=True):
        box('/World/Construction/' + name, x, y, z, sx, sy, sz, color, collision=solid)

    def cylinder(name, x, y, z, radius, height, color, solid=True):
        prim = UsdGeom.Cylinder.Define(stage, '/World/Construction/' + name)
        prim.CreateRadiusAttr(radius)
        prim.CreateHeightAttr(height)
        prim.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdGeom.XformCommonAPI(prim).SetTranslate((x, y, z))
        if solid:
            UsdPhysics.CollisionAPI.Apply(prim.GetPrim())

    cube('Terrain/SandPad', 25, 25, 0, 50, 50, .2, sand)
    cube('Roads/South', 25, 5, .115, 45, 3.0, .03, (.29, .30, .29))
    cube('Roads/East', 45, 25, .12, 3.0, 40, .04, (.29, .30, .29))
    cube('Roads/North', 25, 43, .115, 40, 3.0, .03, (.29, .30, .29))
    cube('Landing/Pad', 2, 2, .125, 2.5, 2.5, .05, (.18, .32, .37))

    # Open concrete frame: slabs, columns, a half-height wall, and steel bars.
    for floor, z in enumerate((.2, 3.0, 5.7)):
        cube(f'Building/Slab_{floor}', 24, 23.5, z, 12, 13, .35, concrete)
    for ix, x in enumerate((18.5, 24.0, 29.5)):
        for iy, y in enumerate((17.5, 23.5, 29.5)):
            cube(f'Building/Column_{ix}_{iy}', x, y, 2.9, .42, .42, 5.8, concrete)
    cube('Building/PartialWall', 24, 29.4, 3.7, 8, .25, 1.1, concrete)
    for i, x in enumerate((20.0, 21.5, 23.0, 24.5)):
        cylinder(f'Building/Rebar_{i}', x, 18, 6.4, .045, 1.3, steel)

    # A scaffold frame next to the building.
    for ix, x in enumerate((32.5, 37.5)):
        for iy, y in enumerate((31.5, 38.5)):
            cylinder(f'Scaffold/Post_{ix}_{iy}', x, y, 2.4, .055, 4.8, steel)
    for level, z in enumerate((1.3, 2.6, 4.0)):
        for iy, y in enumerate((31.5, 38.5)):
            cube(f'Scaffold/RailX_{level}_{iy}', 35, y, z, 5.1, .09, .09, steel)
        for ix, x in enumerate((32.5, 37.5)):
            cube(f'Scaffold/RailY_{level}_{ix}', x, 35, z, .09, 7.1, .09, steel)
    cube('Scaffold/Deck', 35, 35, 2.6, 5.1, 7.1, .1, (.48, .37, .22))

    # Equipment and storage zones.
    cube('Storage/Container', 11, 33, 1.35, 8, 4, 2.7, (.16, .32, .49))
    for i in range(4):
        cube(f'Storage/ContainerRib_{i}', 7.8 + i * 1.9, 30.97, 1.35,
             .13, .08, 2.5, (.10, .22, .37), False)
    for i, x in enumerate((9.0, 11.0, 13.0)):
        cube(f'Materials/Pallet_{i}', x, 9.5, .1, 1.5, 1.8, .2, (.40, .24, .12))
        for j in range(3):
            cube(f'Materials/Block_{i}_{j}', x, 9.5, .32 + .28 * j,
                 1.3, 1.5, .26, concrete)
    cube('Excavation/Pit', 11, 20.5, .11, 6, 7, .02, (.21, .16, .12), False)
    for side, y in enumerate((16.6, 24.4)):
        cube(f'Excavation/Barrier_{side}', 11, y, .55, 7, .13, 1.1, orange)

    # Tower crane and a short jib, all collidable and kept out of the route.
    for i, z in enumerate((1.5, 4.5, 7.5)):
        cube(f'Crane/Mast_{i}', 40.5, 12.5, z, .7, .7, 3, yellow)
    cube('Crane/Jib', 36.0, 12.5, 9.1, 13, .42, .42, yellow)
    cube('Crane/Counterweight', 41.8, 12.5, 8.6, 1.4, 1.4, .9, steel)
    cylinder('Crane/HookCable', 32.0, 12.5, 7.7, .035, 2.4, steel, False)
    cube('Crane/Hook', 32, 12.5, 6.4, .45, .45, .25, orange)

    # Edge barriers read clearly from the air but leave the site open above.
    for side, y in enumerate((.35, 49.65)):
        for i in range(10):
            cube(f'Fence/Y{side}_{i}', 2.5 + i * 5, y, .55,
                 4.7, .13, 1.1, orange)
    for side, x in enumerate((.35, 49.65)):
        for i in range(10):
            cube(f'Fence/X{side}_{i}', x, 2.5 + i * 5, .55,
                 .13, 4.7, 1.1, orange)
