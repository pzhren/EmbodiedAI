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
from simulator.scenes import Interactive_Scene

config_file= "/data1/lfwj/linmin_embodiedAI/EmbodiedAI/tests/test_configs/test.yaml"
cfg = EnvConfig(config_file)
print(cfg.config)
env = BaseEnv(cfg)
# map_path = "/data1/lfwj/hssd_scenes/final_selected_usd/102344022/w_350h_453r_0.050000X_-20.575000381469728Y_-4.875000095367431.png"
i = 0
while env.is_running:
        objs_xformprim = env.sim.find_object_by_id(env.scenes[0], cfg.config.task.task_path)
        goal_pos1, _ = objs_xformprim[0].get_world_pose()
        print("goal_pos1", goal_pos1)
        # 这里很奇怪，要全部加负号才正常
        goal_pose1 = [-goal_pos1[0], -goal_pos1[1]]
        distance, action = env.task.get_distance(goal_pose1)
        print("distance, action", distance, action)
        obs = env.step([action])
        # 把obs的观测数据按照图片保存下来
        rgb1 = obs[0][0][0]["rgb"]
        rgb2 = obs[0][0][1]["rgb"]
        rgb3 = obs[0][0][2]["rgb"]
        
        # depth1 = obs[0][0][0]["depth"]
        # depth2 = obs[0][0][1]["depth"]
        # depth3 = obs[0][0][2]["depth"]
        
        # 将这些图片保存下来，利用Pillow库
        from PIL import Image
        Image.fromarray(rgb1).save(f"/data1/lfwj/linmin_embodiedAI/EmbodiedAI/tests/obs/lf_2/rgb1_{i}.png")
        Image.fromarray(rgb2).save(f"/data1/lfwj/linmin_embodiedAI/EmbodiedAI/tests/obs/lf_2/rgb2_{i}.png")
        Image.fromarray(rgb3).save(f"/data1/lfwj/linmin_embodiedAI/EmbodiedAI/tests/obs/lf_2/rgb3_{i}.png")
        i+=1