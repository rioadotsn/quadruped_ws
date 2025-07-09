import numpy as np
import threading
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from go2_policycontroller import Go2PolicyController
from isaacsim.storage.native import get_assets_root_path



first_step = True
reset_needed = False
base_command = np.zeros(3)
running = True
command_lock = threading.Lock()

def input_thread():
    global base_command, running
    print("輸入 w/a/s/d/q/e, quit退出")
    
    while running:
        try:
            key = input().strip().lower()
            if key == 'quit': 
                running = False
                break
            
            new_command = np.zeros(3)
            if 'w' in key: new_command[0] += 1.0
            if 's' in key: new_command[0] -= 1.0
            if 'a' in key: new_command[1] += 1.0
            if 'd' in key: new_command[1] -= 1.0
            if 'q' in key: new_command[2] += 1.0
            if 'e' in key: new_command[2] -= 1.0
            
            with command_lock:
                base_command[:] = new_command
        except:
            running = False
            break

def on_physics_step(step_size):
    global first_step, reset_needed
    if first_step:
        go2.initialize()
        first_step = False
    elif reset_needed:
        my_world.reset(True)
        go2.initialize()
        reset_needed = False
        first_step = True
    else:
        with command_lock:
            current_command = base_command.copy()
        go2.forward(step_size, current_command)

my_world = World(stage_units_in_meters=1.0, physics_dt=1/200, rendering_dt=1/50)
assets_root_path = get_assets_root_path()

prim = define_prim("/World/Ground", "Xform")
asset_path = assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
prim.GetReferences().AddReference(asset_path)

go2 = Go2PolicyController(
    prim_path="/World/Go2",
    name="Go2",
    position=np.array([0, 0, 0.5]),
)

my_world.reset()
my_world.add_physics_callback("physics_step", callback_fn=on_physics_step)

threading.Thread(target=input_thread, daemon=True).start()

while simulation_app.is_running() and running:
    my_world.step(render=True)
    if my_world.is_stopped():
        reset_needed = True

simulation_app.close()