# UR10機器人最小控制版本
# 資料來源：基於原始程式碼簡化而來

from isaacsim import SimulationApp

# 1. 初始化Isaac Sim（必須在其他import之前）
# 資料來源：Isaac Sim官方文檔要求
simulation_app = SimulationApp({"headless": False})

import numpy as np
import time

# 2. Isaac Sim相關模組（必須在SimulationApp之後）
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api import World

class UR10Controller:
    """UR10機器人簡化控制器"""
    
    def __init__(self):
        self.world = None
        self.robot = None
        
        # 任務狀態（推論：簡化的狀態機）
        self.current_step = 0
        self.is_moving = False
        self.step_start_time = 0.0
        self.move_duration = 5.0  # 每步運動時間（秒）
        
        # 目標關節角度列表（資料來源：原程式碼的target_positions）
        # 格式：[關節1, 關節2, 關節3, 關節4, 關節5, 關節6]（弧度）
        pi = np.pi
        self.target_angles = [
            [0.0, -120*pi/180, 120*pi/180, 90*pi/180, 90*pi/180, 0.0],              # 初始位置
            [10*pi/180, -43*pi/180, 75*pi/180, 60*pi/180, 90*pi/180, -80*pi/180],
            [80*pi/180, -43*pi/180, 75*pi/180, 60*pi/180, 90*pi/180, 0*pi/180],
            [80*pi/180, -40*pi/180, 75*pi/180, 60*pi/180, 90*pi/180, 90*pi/180],
            [120*pi/180, -43*pi/180, 75*pi/180, 60*pi/180, 90*pi/180, -150*pi/180],
        ]
        
        # 軌跡控制變數（推論：用於平滑運動）
        self.start_positions = None
        self.target_positions = None
        
    def setup_scene(self):
        """設置場景和機器人"""
        print("設置場景...")
        
        # 建立世界（資料來源：Isaac Sim API）
        self.world = World()
        self.world.scene.add_default_ground_plane()
        
        # 載入機器人模型（資料來源：原程式碼路徑）
        robot_path = "/home/daylight-ubuntu/USDfile/ur10_wrench_fla.usd"
        prim_path = "/World/ur10_wrench_fla/ur10_robotiq_official_flattened"
        
        # 檢查檔案存在性（推論：基本錯誤處理）
        import os
        if not os.path.exists(robot_path):
            print(f"錯誤：找不到機器人檔案 {robot_path}")
            return False
            
        # 載入並初始化機器人
        add_reference_to_stage(usd_path=robot_path, prim_path=prim_path)
        self.robot = Articulation(prim_paths_expr=prim_path, name="ur10")
        self.world.scene.add(self.robot)
        self.world.reset()
        
        print(f"機器人載入完成，關節數：{self.robot.num_dof}")
        return True
        
    def start_next_movement(self):
        """開始下一個運動步驟"""
        if self.current_step >= len(self.target_angles):
            print("所有步驟完成！")
            return
            
        # 獲取當前位置作為起點（推論：確保軌跡連續性）
        try:
            current_pos = self.robot.get_joint_positions()
            if len(current_pos.shape) > 1:
                self.start_positions = current_pos[0]
            else:
                self.start_positions = current_pos
        except:
            print("警告：無法獲取當前位置")
            self.start_positions = np.zeros(self.robot.num_dof)
        
        # 準備目標位置（推論：擴展6個UR10關節到所有關節）
        target_ur10 = self.target_angles[self.current_step]
        self.target_positions = np.zeros(self.robot.num_dof)
        self.target_positions[:6] = target_ur10  # 前6個是UR10關節
        
        # 開始運動
        self.is_moving = True
        self.step_start_time = self.world.current_time
        print(f"步驟 {self.current_step + 1}: 移動到 {[f'{a:.2f}' for a in target_ur10]}")
        
    def update_movement(self):
        """更新運動狀態"""
        if not self.is_moving:
            return
            
        # 計算運動進度（資料來源：線性插值數學公式）
        elapsed = self.world.current_time - self.step_start_time
        progress = min(elapsed / self.move_duration, 1.0)
        
        # 線性插值計算當前位置（推論：t∈[0,1]的線性插值）
        # 公式：current = start + progress * (target - start)
        current_positions = (1 - progress) * self.start_positions + progress * self.target_positions
        
        # 應用到機器人
        self.robot.set_joint_positions([current_positions])
        
        # 檢查是否完成當前步驟
        if progress >= 1.0:
            self.is_moving = False
            self.current_step += 1
            print(f"步驟完成，等待1秒...")
            time.sleep(1)  # 簡單的等待（推論：讓機器人穩定）
            
    def reset(self):
        """重置控制器狀態"""
        self.current_step = 0
        self.is_moving = False
        print("控制器已重置")
        
    def step(self):
        """每幀更新函數"""
        # 如果沒有在運動且還有步驟，開始下一步
        if not self.is_moving and self.current_step < len(self.target_angles):
            self.start_next_movement()
        
        # 更新當前運動
        self.update_movement()

def main():
    """主函數"""
    controller = None
    
    try:
        print("啟動UR10控制程式...")
        
        # 建立並設置控制器
        controller = UR10Controller()
        if not controller.setup_scene():
            return
            
        print(f"準備執行 {len(controller.target_angles)} 個步驟")
        print("按 ▶️ 開始，按 ⏹️ 停止")
        
        # 主循環（資料來源：Isaac Sim標準模式）
        was_playing = False
        
        while simulation_app.is_running():
            controller.world.step(render=True)
            
            is_playing = controller.world.is_playing()
            
            # 檢測播放狀態變化（推論：用於觸發重置）
            if is_playing and not was_playing:
                print("開始播放 - 重置任務")
                controller.world.reset()
                controller.reset()
            
            # 只在播放時更新控制器
            if is_playing:
                controller.step()
                
            was_playing = is_playing
        
    except KeyboardInterrupt:
        print("程式被中斷")
    except Exception as e:
        print(f"錯誤：{e}")
    finally:
        # 清理資源（資料來源：Isaac Sim官方清理程序）
        if controller and controller.world:
            controller.world.clear_instance()
        simulation_app.close()
        print("程式結束")

if __name__ == "__main__":
    main()
