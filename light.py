from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import omni.usd
import omni.appwindow
import numpy as np
from pxr import UsdGeom, Sdf, Gf

# 建立場景
omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()

# 建立 DomeLight
dome_light_path = "/World/DomeLight"
dome_light = stage.DefinePrim(dome_light_path, "DomeLight")
intensity_attr = dome_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float)
color_attr = dome_light.CreateAttribute("inputs:color", Sdf.ValueTypeNames.Color3f)

# 預設光源參數
light_settings = [
    {"intensity": 900.0, "color": Gf.Vec3f(1.0, 1.0, 1.0)},
    {"intensity": 3000.0, "color": Gf.Vec3f(1.0, 0.8, 0.6)},
    {"intensity": 100.0, "color": Gf.Vec3f(0.4, 0.4, 1.0)},
]
current_setting = 0
intensity_attr.Set(light_settings[current_setting]["intensity"])
color_attr.Set(light_settings[current_setting]["color"])

# 加入 Cube
cube_prim = stage.DefinePrim("/World/Cube", "Cube")
UsdGeom.Xformable(cube_prim).AddTranslateOp().Set(Gf.Vec3f(0.0, 5.0, 1.0))

# 加入地面
plane_prim = stage.DefinePrim("/World/Plane", "Xform")
plane_geom = UsdGeom.Mesh.Define(stage, "/World/Plane/Geom")
plane_geom.CreatePointsAttr([
    (-1000, -1000, 0),
    (1000, -1000, 0),
    (1000, 1000, 0),
    (-1000, 1000, 0)
])
plane_geom.CreateFaceVertexCountsAttr([4])
plane_geom.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
plane_geom.CreateExtentAttr([(-1000, -1000, 0), (1000, 1000, 0)])

# 鍵盤監聽器註冊
appwindow = omni.appwindow.get_default_app_window()
input_iface = carb.input.acquire_input_interface()
keyboard = appwindow.get_keyboard()

def keyboard_event_callback(event, *args, **kwargs):
    global current_setting
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        if event.input.name == "Q":
            current_setting = (current_setting + 1) % len(light_settings)
            intensity_attr.Set(light_settings[current_setting]["intensity"])
            color_attr.Set(light_settings[current_setting]["color"])
            print(f"[INFO] Switched to setting {current_setting}")
    return True

# 訂閱鍵盤事件
keyboard_sub = input_iface.subscribe_to_keyboard_events(keyboard, keyboard_event_callback)

# 主循環
while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()

