import os
os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"

# import isaacsim
# from omni.isaac.kit import SimulationApp

# simulation_app = SimulationApp({"headless": False})
import isaacsim
from omni.isaac.kit import SimulationApp

config = {
     "width": "1280",
     "height": "720",
     "headless": False,
}
simulation_app = SimulationApp(config)

import carb
import numpy as np
import time
from omni.isaac.core import World
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import RigidPrim,RigidPrimView,GeometryPrim
from omni.isaac.core.robots import Robot
import omni.isaac.core.utils.prims as prim_utils
# from omni.isaac.core.utils.prims import create_prim
from omni.isaac.sensor import Camera,RotatingLidarPhysX,IMUSensor
from omni.isaac.core.scenes import Scene as OmniBaseScene
import matplotlib.pyplot as plt
import omni.kit.viewport.utility as vu 
# import omni.isaac.core.utils.viewports_utils as vu
# from omni.isaac.core.utils import render_product_utils
# from .devices import Keyboard
# Rest of the code follows

my_world = World(stage_units_in_meters=1.0)

robot = Robot(prim_path="/World/robot",usd_path="/data1/linmin/EmbodiedAI/tests/stretch/model/stretch.usd")

my_world.scene.add(robot)
my_world.reset()

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
        
simulation_app.close()