import sys
import time
import math

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})  # 使用 GUI 模式

import carb
import omni
from isaacsim.storage.native import get_assets_root_path
from omni.isaac.dynamic_control import _dynamic_control

# 載入場景
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("找不到 Isaac Sim 資源路徑")
    simulation_app.close()
    sys.exit(1)

asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
usd_ctx = omni.usd.get_context()
usd_ctx.open_stage(asset_path)

# 等待 stage 載入完成
while not usd_ctx.get_stage():
    simulation_app.update()
    time.sleep(0.1)

print("Stage loaded successfully.")

# 啟動模擬
timeline = omni.timeline.get_timeline_interface()
timeline.play()

while not timeline.is_playing():
    simulation_app.update()
    time.sleep(0.05)

# 執行一次 step 讓 scene 初始化
simulation_app.update()

# 取得 articulation
dc = _dynamic_control.acquire_dynamic_control_interface()
art = dc.get_articulation("/panda")

if art == _dynamic_control.INVALID_HANDLE:
    print("ERROR: '/panda' is not a valid articulation")
    simulation_app.close()
    sys.exit(1)

print(f"Got articulation handle: {art}")

# 找到要控制的關節
joint_name = "panda_finger_joint1"
dof_ptr = dc.find_articulation_dof(art, joint_name)
if dof_ptr == _dynamic_control.INVALID_HANDLE:
    print(f"ERROR: DOF '{joint_name}' not found.")
    simulation_app.close()
    sys.exit(1)

# 主迴圈：讓關節持續來回擺動
print("開始控制 panda_joint2（sin 波）... 按 Ctrl+C 或關閉視窗結束")

t_start = time.time()
try:
    while simulation_app.is_running() and timeline.is_playing():
        t_now = time.time() - t_start
        angle = 0.5 * math.sin(0.5 * t_now)  # 在 -1.5 ~ 1.5 範圍擺動
        dc.wake_up_articulation(art)
        dc.set_dof_position_target(dof_ptr, angle)
        simulation_app.update()
except KeyboardInterrupt:
    print("中斷模擬，關閉應用程式...")

# 清理資源
simulation_app.close()

