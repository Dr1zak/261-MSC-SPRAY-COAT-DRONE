"""Visual X650-class frame surrogate; source X500 sprayer links remain imported."""
import math


def author_proxy(stage, base_path, config):
    from pxr import Gf, UsdGeom

    original = UsdGeom.Imageable(stage.GetPrimAtPath(base_path + '/x500_frame'))
    if original:
        original.MakeInvisible()
    rotor_positions = config['rotor_positions_m']
    color = Gf.Vec3f(.08, .11, .13)

    def cube(name, xyz, scale, yaw=0):
        prim = UsdGeom.Cube.Define(stage, base_path + '/X650Study/' + name)
        prim.CreateSizeAttr(1.0)
        prim.CreateDisplayColorAttr([color])
        xf = UsdGeom.XformCommonAPI(prim)
        xf.SetTranslate(tuple(xyz))
        xf.SetRotate((0.0, 0.0, float(yaw)))
        xf.SetScale(tuple(scale))

    cube('CentralBody', (0, 0, 0), (.16, .16, .03))
    for index, (x, y, z) in enumerate(rotor_positions, 1):
        arm_length = math.hypot(x, y)
        cube(f'Arm_{index}', (x/2, y/2, z-.018), (arm_length, .025, .022),
             math.degrees(math.atan2(y, x)))
        motor = UsdGeom.Cylinder.Define(stage, base_path + f'/X650Study/Motor_{index}')
        motor.CreateRadiusAttr(.023)
        motor.CreateHeightAttr(.034)
        motor.CreateDisplayColorAttr([Gf.Vec3f(.19, .21, .22)])
        UsdGeom.XformCommonAPI(motor).SetTranslate((x, y, z))
        disk = UsdGeom.Cylinder.Define(stage, base_path + f'/X650Study/PropDisk_{index}')
        disk.CreateRadiusAttr(config['propeller_diameter_m']/2)
        disk.CreateHeightAttr(.002)
        disk.CreateDisplayColorAttr([Gf.Vec3f(.27, .33, .35)])
        disk.CreateDisplayOpacityAttr([.23])
        UsdGeom.XformCommonAPI(disk).SetTranslate((x, y, z+.02))
