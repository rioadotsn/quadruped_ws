# UR10機器人鍵盤控制版本
# 控制說明：
# Q/W: 關節1正轉/反轉
# R/T: 關節2正轉/反轉
# B/M: 關節3正轉/反轉
# Z/X: 關節4正轉/反轉
# J/K: 關節5正轉/反轉
# L: 關節6正轉
# SPACE: 重置到初始位置

from isaacsim import SimulationApp

# 1. 初始化Isaac Sim（必須在其他import之前）
simulation_app = SimulationApp({"headless": False})

import carb
import carb.input
import numpy as np
import omni.appwindow  # Contains handle to keyboard

# 2. Isaac Sim相關模組（必須在SimulationApp之後）
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api import World

class UR10KeyboardController:
    """UR10機器人鍵盤控制器"""
    
    def __init__(self, physics_dt, render_dt):
        self._world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=render_dt)
        self.robot = None
        
        # 控制參數
        self.move_speed = 0.02  # 移動速度（弧度/幀）
        self.joint6_speed = 0.05  # 關節6旋轉速度（弧度/幀）
        
        # 初始關節角度（弧度）
        pi = np.pi
        self.initial_angles = [0.0, -120*pi/180, 120*pi/180, 90*pi/180, 90*pi/180, 0.0]
        
        # 當前目標位置
        self.current_positions = None
        
        # 關節控制命令 [joint1, joint2, joint3, joint4, joint5, joint6]
        self._joint_commands = np.zeros(6)
        
        # 關節限制（弧度）
        self.joint_limits = [
            (-2*pi, 2*pi),      # 關節1: 基座旋轉
            (-pi, 0),           # 關節2: 肩膀上下
            (-pi, pi),          # 關節3: 手肘
            (-pi, pi),          # 關節4: 手腕1
            (-pi, pi),          # 關節5: 手腕2
            (-2*pi, 2*pi)       # 關節6: 手腕3（工具旋轉）
        ]
        
        # 鍵盤映射 - 使用QWRTBM按鍵
        self._input_keyboard_mapping = {
            # 關節1控制 (Q/W)
            "Q": [0.05, 0.0, 0.0, 0.0, 0.0, 0.0],        # 關節1正轉
            "W": [-0.05, 0.0, 0.0, 0.0, 0.0, 0.0],       # 關節1反轉
            
            # 關節2控制 (R/T)
            "R": [0.0, 0.05, 0.0, 0.0, 0.0, 0.0],        # 關節2正轉
            "T": [0.0, -0.05, 0.0, 0.0, 0.0, 0.0],       # 關節2反轉
            
            # 關節3控制 (B/M)
            "B": [0.0, 0.0, 0.05, 0.0, 0.0, 0.0],        # 關節3正轉
            "M": [0.0, 0.0, -0.05, 0.0, 0.0, 0.0],       # 關節3反轉
            
            # 關節4控制 (Z/X)
            "Z": [0.0, 0.0, 0.0, 0.05, 0.0, 0.0],        # 關節4正轉
            "X": [0.0, 0.0, 0.0, -0.05, 0.0, 0.0],       # 關節4反轉
            
            # 關節5控制 (J/K)
            "J": [0.0, 0.0, 0.0, 0.0, 0.05, 0.0],        # 關節5正轉
            "K": [0.0, 0.0, 0.0, 0.0, -0.05, 0.0],       # 關節5反轉
            
            # 關節6控制 (L)
            "L": [0.0, 0.0, 0.0, 0.0, 0.0, 0.05],        # 關節6正轉
        }
        
        self.needs_reset = False
        self.first_step = True
        
    def setup_scene(self):
        """設置場景和機器人"""
        print("設置場景...")
        
        # 建立世界並添加地面
        self._world.scene.add_default_ground_plane()
        
        # 載入機器人模型
        robot_path = "/home/daylight-ubuntu/USDfile/ur10_wrench_fla.usd"
        prim_path = "/World/ur10_wrench_fla/ur10_robotiq_official_flattened"
        
        # 檢查檔案存在性
        import os
        if not os.path.exists(robot_path):
            print(f"錯誤：找不到機器人檔案 {robot_path}")
            return False
            
        # 載入並初始化機器人
        add_reference_to_stage(usd_path=robot_path, prim_path=prim_path)
        self.robot = Articulation(prim_paths_expr=prim_path, name="ur10")
        self._world.scene.add(self.robot)
        
        print(f"機器人載入完成，關節數：{self.robot.num_dof}")
        print("\n=== 控制說明 ===")
        print("Q/W: 關節1正轉/反轉")
        print("R/T: 關節2正轉/反轉")
        print("B/M: 關節3正轉/反轉")
        print("Z/X: 關節4正轉/反轉")
        print("J/K: 關節5正轉/反轉")
        print("L: 關節6正轉")
        print("SPACE: 重置到初始位置")
        print("================\n")
        
        return True
    
    def setup_keyboard(self):
        """設置鍵盤監聽器 - 根據參考程式"""
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)
        self._world.add_physics_callback("ur10_control", callback_fn=self.on_physics_step)
    
    def reset_to_initial_position(self):
        """重置到初始位置"""
        if self.current_positions is None:
            self.current_positions = np.zeros(self.robot.num_dof)
        self.current_positions[:6] = self.initial_angles
        self.robot.set_joint_positions([self.current_positions])
        print("機器人已重置到初始位置")
    
    def clamp_joint_angle(self, joint_idx, angle):
        """限制關節角度在合理範圍內"""
        if joint_idx < len(self.joint_limits):
            min_angle, max_angle = self.joint_limits[joint_idx]
            return np.clip(angle, min_angle, max_angle)
        return angle
    
    def on_physics_step(self, step_size):
        """物理步驟回調函數 - 根據參考程式"""
        if self.first_step:
            # 初始化機器人
            self.robot.initialize()
            self.reset_to_initial_position()
            self.first_step = False
        elif self.needs_reset:
            self._world.reset(True)
            self.needs_reset = False
            self.first_step = True
        else:
            # 更新機器人位置
            self.update_robot_position()
    
    def update_robot_position(self):
        """根據命令更新機器人位置"""
        if self.current_positions is None:
            self.current_positions = np.zeros(self.robot.num_dof)
            self.current_positions[:6] = self.initial_angles
        
        # 獲取當前位置
        try:
            current_joint_positions = self.robot.get_joint_positions()
            if len(current_joint_positions.shape) > 1:
                self.current_positions = current_joint_positions[0].copy()
            else:
                self.current_positions = current_joint_positions.copy()
        except:
            pass
        
        # 根據命令更新位置
        position_changed = False
        
        # 關節1控制
        if self._joint_commands[0] != 0:
            new_angle = self.current_positions[0] + self._joint_commands[0] * self.move_speed
            self.current_positions[0] = self.clamp_joint_angle(0, new_angle)
            position_changed = True
        
        # 關節2控制
        if self._joint_commands[1] != 0:
            new_angle = self.current_positions[1] + self._joint_commands[1] * self.move_speed
            self.current_positions[1] = self.clamp_joint_angle(1, new_angle)
            position_changed = True
        
        # 關節3控制
        if self._joint_commands[2] != 0:
            new_angle = self.current_positions[2] + self._joint_commands[2] * self.move_speed
            self.current_positions[2] = self.clamp_joint_angle(2, new_angle)
            position_changed = True
        
        # 關節4控制
        if self._joint_commands[3] != 0:
            new_angle = self.current_positions[3] + self._joint_commands[3] * self.move_speed
            self.current_positions[3] = self.clamp_joint_angle(3, new_angle)
            position_changed = True
        
        # 關節5控制
        if self._joint_commands[4] != 0:
            new_angle = self.current_positions[4] + self._joint_commands[4] * self.move_speed
            self.current_positions[4] = self.clamp_joint_angle(4, new_angle)
            position_changed = True
        
        # 關節6控制
        if self._joint_commands[5] != 0:
            new_angle = self.current_positions[5] + self._joint_commands[5] * self.joint6_speed
            self.current_positions[5] = self.clamp_joint_angle(5, new_angle)
            position_changed = True
        
        # 如果位置有變化，更新機器人
        if position_changed:
            self.robot.set_joint_positions([self.current_positions])
    
    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        """鍵盤事件回調函數 - 根據參考程式"""
        
        # 處理重置鍵
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name == "SPACE":
                self.reset_to_initial_position()
                return True
        
        # 處理移動控制鍵
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            # 按下時，增加命令
            if event.input.name in self._input_keyboard_mapping:
                self._joint_commands += np.array(self._input_keyboard_mapping[event.input.name])
        
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            # 釋放時，減少命令
            if event.input.name in self._input_keyboard_mapping:
                self._joint_commands -= np.array(self._input_keyboard_mapping[event.input.name])
        
        return True
    
    def run(self):
        """運行主循環 - 根據參考程式"""
        while simulation_app.is_running():
            self._world.step(render=True)
            if self._world.is_stopped():
                self.needs_reset = True

def main():
    """主函數"""
    try:
        print("啟動UR10鍵盤控制程式...")
        
        # 設置物理和渲染頻率
        physics_dt = 1 / 200.0
        render_dt = 1 / 60.0
        
        # 建立控制器
        controller = UR10KeyboardController(physics_dt=physics_dt, render_dt=render_dt)
        
        # 設置場景
        if not controller.setup_scene():
            return
        
        # 更新應用程式
        simulation_app.update()
        
        # 重置世界
        controller._world.reset()
        simulation_app.update()
        
        # 設置鍵盤監聽
        controller.setup_keyboard()
        simulation_app.update()
        
        print("鍵盤監聽已啟動，可以開始控制機器人")
        print("請點擊播放按鈕開始控制")
        
        # 運行主循環
        controller.run()
        
    except KeyboardInterrupt:
        print("程式被中斷")
    except Exception as e:
        print(f"錯誤：{e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理資源
        simulation_app.close()
        print("程式結束")

if __name__ == "__main__":
    main()