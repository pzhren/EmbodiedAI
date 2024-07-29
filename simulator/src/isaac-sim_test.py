# import asyncio
# import os

# import omni.kit.app
# from omni.isaac.kit import AppFramework
# import time
# argv = [
#     "--empty",
#     "--ext-folder",
#     f'{os.path.abspath(os.environ["ISAAC_PATH"])}/exts',
#     "--/app/asyncRendering=False",
#     "--/app/fastShutdown=False",
#     "--enable",
#     "omni.usd",
#     "--enable",
#     "omni.kit.uiapp",
# ]
# # startup
# app = AppFramework("test_app", argv)

# import omni.usd

# stage_task = asyncio.ensure_future(omni.usd.get_context().new_stage_async())

# while not stage_task.done():
#     app.update()

# print("exiting")

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
from omni.isaac.sensor import Camera,RotatingLidarPhysX,IMUSensor
import matplotlib.pyplot as plt

# from .devices import Keyboard
# Rest of the code follows

my_world = World(stage_units_in_meters=1.0)
assets_root_path = get_assets_root_path()

if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"

my_jetbot = WheeledRobot(
        prim_path="/World/Jetbot",
        name="my_jetbot",
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
        create_robot=True,
        usd_path=jetbot_asset_path,
        position=np.array([1.0, 0.5, 1.0]),
    )
my_world.scene.add(my_jetbot)
camera = Camera("/World/Jetbot/chassis/rgb_camera/jetbot_camera",resolution=(1280,640))
my_lidar = my_world.scene.add(
    RotatingLidarPhysX(
        prim_path="/World/Jetbot/chassis/lidar", name="lidar", translation=np.array([0, 0, 0.1])
    )
)
imu_sensor_left = my_world.scene.add(
    IMUSensor(
        prim_path="/World/Jetbot/left_wheel/imu_sensor_left",
        name="imu_left",
        frequency=60,
        translation=np.array([0, 0, 0]),
    )
)
imu_sensor_right = my_world.scene.add(
    IMUSensor(
        prim_path="/World/Jetbot/right_wheel/imu_sensor_right",
        name="imu_right",
        frequency=60,
        translation=np.array([0, 0, 0]),
    )
)
light_1 = prim_utils.create_prim(
    "/World/Light_1",
    "SphereLight",
    position=np.array([0.0, 0.0, 3.5]),
    attributes={
        "inputs:radius": 0.5,
        "inputs:intensity": 1e5,
        "inputs:color": (1.0, 1.0, 1.0)
    }
)
reconstruct_scene_path = "D://MMLM_Robot//Grasp_Nav//code//simulator//reconstruct_scene//scene_usd//playroom.usd"
my_world.scene.add_default_ground_plane()

construct_scene = add_reference_to_stage(usd_path=reconstruct_scene_path, prim_path="/World/Scene")

robot = Robot("/World/Jetbot")
robot.set_local_scale(np.array([5.0,5.0,5.0]))

Plane = RigidPrim("/World/defaultGroundPlane")
Plane.set_visibility(visible=False)
Plane.disable_rigid_body_physics()

# Plane.set_

construct_scene = GeometryPrim("/World/Scene",collision=True)
construct_scene.set_local_pose(translation=np.array([1.19348, 1.37576, 1.95]), orientation=np.array([0.19885, 0.98, 0., 0.]))
construct_scene.set_local_scale(np.array([0.5, 0.5, 0.5]))

# construct_scene.disable_rigid_body_physics()

my_controller = DifferentialController(name="simple_control", wheel_radius=0.03, wheel_base=0.1125)
my_world.reset()
i = 0
reset_needed = False

camera.initialize()
camera.add_motion_vectors_to_frame()
my_lidar.add_depth_data_to_frame()
my_lidar.add_point_cloud_data_to_frame()
my_lidar.enable_visualization()

# robot_input_limit=[[-1,-1],[1,1]]
# robot_output_limit=[[-0.05,-1*np.pi/2],[0.05,np.pi/2]]

from devices.keyboard import Keyboard
device = Keyboard()

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            reset_needed = False
        # print(camera.get_resolution())
        # time.sleep(0.1)
        if i >= 0 and i < 1000:
            # forward
            action = device.get_controller_state()
            # print(action)
            my_jetbot.apply_wheel_actions(my_controller.forward(command=action))
        # print(my_lidar.get_current_frame())
        print("left:",imu_sensor_left.get_current_frame())
        print("right:",imu_sensor_left.get_current_frame())
        
        if i > 100:
            plt.imsave("D://MMLM_Robot//Grasp_Nav//code//simulator//obs//rgb//rgb_obs"+str(i)+".png",camera.get_rgb())
            plt.imsave("D://MMLM_Robot//Grasp_Nav//code//simulator//obs//depth//rgb_obs"+str(i)+".png",camera.get_depth())
        i += 1


simulation_app.close()

