# import isaacsim
# from omni.isaac.kit import SimulationApp


# config = {
#      "width": "1280",
#      "height": "720",
#      "headless": False,
# }
# simulation_app = SimulationApp(config)

# import carb
# import numpy as np
# import time
# from omni.isaac.core import World
# from omni.isaac.nucleus import get_assets_root_path
# from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
# from omni.isaac.wheeled_robots.robots import WheeledRobot
# from omni.isaac.core.utils.stage import add_reference_to_stage
# from omni.isaac.core.prims import RigidPrim,RigidPrimView,GeometryPrim
# from omni.isaac.core.robots import Robot
# import omni.isaac.core.utils.prims as prim_utils
# from omni.isaac.sensor import Camera,RotatingLidarPhysX,IMUSensor
# import matplotlib.pyplot as plt
# import pxr
# reset_needed = False
# usd_path ="D://MMLM_Robot//Grasp_Nav//Simulator//refined_mesh//wall_with_door_overrides.usd"
# my_world = World(stage_units_in_meters=1.0)

# load_obj = add_reference_to_stage(usd_path=usd_path, prim_path="/World/wall_with_door")
# my_world.scene.add_default_ground_plane()
# my_world.reset()

# while simulation_app.is_running():
#     my_world.step(render=True)
#     if my_world.is_stopped() and not reset_needed:
#         reset_needed = True
#     if my_world.is_playing():
#         if reset_needed:
#             my_world.reset()
#             my_controller.reset()
#             reset_needed = False
from simulator.core.config import EnvConfig
from simulator.core.env import BaseEnv

config_file= "D:\MMLM_Robot\Grasp_Nav\code\EmbodiedAI\simulator\configs//test.yaml"
cfg = EnvConfig(config_file)
print(cfg.config)
env = BaseEnv(cfg)