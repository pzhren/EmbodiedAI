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
        position=np.array([1.6, 1.6, 0.5]),
    )
my_jetbot = my_world.scene.add(my_jetbot)

camera = Camera("/World/Jetbot/chassis/rgb_camera/jetbot_camera",resolution=(1280,640))



vision_camera = Camera("/World/vison_camera",
                    position = np.array([0.0, 0.0, 0.0]),
                    resolution=(1280,640),
                    frequency=20,
                    orientation = np.array([0, 0, 0,1]))

# viewport = vu.create_viewport_window()
# viewport.viewport_api.set_active_camera("/World/vison_camera")
# vp_api = vu.get_active_viewport()
# print(vu.get_active_viewport_and_window())
# vp_api.camera_path = "/World/vision_camera"



# my_lidar = my_world.scene.add(
#     RotatingLidarPhysX(
#         prim_path="/World/Jetbot/chassis/lidar", name="lidar", translation=np.array([0, 0, 0.1])
#     )
# )
# imu_sensor_left = my_world.scene.add(
#     IMUSensor(
#         prim_path="/World/Jetbot/left_wheel/imu_sensor_left",
#         name="imu_left",
#         frequency=60,
#         translation=np.array([0, 0, 0]),
#     )
# )
# imu_sensor_right = my_world.scene.add(
#     IMUSensor(
#         prim_path="/World/Jetbot/right_wheel/imu_sensor_right",
#         name="imu_right",
#         frequency=60,
#         translation=np.array([0, 0, 0]),
#     )
# )
light_1 = prim_utils.create_prim(
    "/World/Light_1",
    "SphereLight",
    # position=np.array([0.0, 0.0, 3.5]),
    position=np.array([0.5, 0.6, 4.2]),
    attributes={
        "inputs:radius": 0.5,
        "inputs:intensity": 1e5,
        "inputs:color": (1.0, 1.0, 1.0)
        
    }
)
my_world.scene.add_default_ground_plane()
construct_scene=prim_utils.create_prim(
    prim_type="Xform",
    prim_path = "/World/Scene",
    usd_path="D:\MMLM_Robot\Grasp_Nav\code//reconstruct_scene\play_room_pgsr.usd",
    # translation=np.array([0.0, 0.0, -0.01]),
    # usd_path="D:\MMLM_Robot\Grasp_Nav\code//reconstruct_scene\drjohn.usd",
)
# reconstruct_scene_path = "D:\MMLM_Robot\Grasp_Nav\code//reconstruct_scene\play_room.usd"

# construct_scene = add_reference_to_stage(usd_path=reconstruct_scene_path, prim_path="/World/Scene")

robot = Robot("/World/Jetbot")
robot.set_local_scale(np.array([2.2,2.2,2.2]))

Plane = RigidPrim("/World/defaultGroundPlane")
Plane.set_visibility(visible=False)
Plane.disable_rigid_body_physics()

# Plane.set_

construct_scene = GeometryPrim("/World/Scene",collision=True)
# construct_scene.set_local_pose(translation=np.array([1.19348, 1.3set_local_pose(translation=np.array([0, 0, -0.03]))
construct_scene.set_local_scale(np.array([1.0, 1.0, 1.0]), orientation=np.array([0.19885, 0.98, 0., 0.]))


# construct_scene.disable_rigid_body_physics()

my_controller = DifferentialController(name="simple_control", wheel_radius=0.03, wheel_base=0.1125)
my_world.reset()
i = 0
reset_needed = False

camera.initialize()
camera.add_motion_vectors_to_frame()
camera.add_distance_to_camera_to_frame()
# my_lidar.add_depth_data_to_frame()
# my_lidar.add_point_cloud_data_to_frame()
# my_lidar.enable_visualization()

# robot_input_limit=[[-1,-1],[1,1]]
# robot_output_limit=[[-0.05,-1*np.pi/2],[0.05,np.pi/2]]

# from simulator.devices.keyboard import Keyboard
# device = Keyboard()
# from omni.ui import Window
# from omni.kit.widget.viewport import ViewportWidget
# viewport_window = Window('SimpleViewport', width=1280, height=720+20) # Add 20 for the title-bar
# with viewport_window.frame:
#     viewport_widget = ViewportWidget(resolution = (1280, 720))

# # Control of the ViewportTexture happens through the object held in the viewport_api property
# viewport_api = viewport_widget.viewport_api

# # We can reduce the resolution of the render easily
# viewport_api.resolution = (640, 480)

# # We can also switch to a different camera if we know the path to one that exists
# viewport_api.camera_path =  "/World/vision_camera"

# vp_api = vu.get_active_viewport()
# print(vu.get_active_viewport_and_window())
# vp_api.camera_path = "/World/vision_camera"

pos_list = []
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
        if i >= 0 and i < 300:
            # forward
            # continue
            position, orientation = my_jetbot.get_local_pose()
            print(position)
            my_jetbot.apply_wheel_actions(my_controller.forward(command=[0.3, np.pi/2]))
        if i >= 300 and i < 500:
            position, orientation = my_jetbot.get_local_pose()
            print(position)
            my_jetbot.apply_wheel_actions(my_controller.forward(command=[0.2, -np.pi/2]))
        if i>= 500:
            position, orientation = my_jetbot.get_local_pose()
            my_jetbot.apply_wheel_actions(my_controller.forward(command=[0.2, np.pi/2]))
            print(position)

        pos_list.append(position.tolist())
            # action = device.get_controller_state()
            # transition, orientaton = vision_camera.get_local_pose()
            # print(action)
            # transition = transition+action[0]
            # print(transition)
            # vision_camera.set_local_pose(
            #     translation = transition
            # )
        if camera.get_rgb().any():
            print(type(camera.get_rgb()))
            print(camera.get_rgb().shape)
            plt.imsave("D:\MMLM_Robot\Grasp_Nav\code\EmbodiedAI//tests//obs//rgb//"+f"frame_{i:03d}.png",camera.get_rgb())
            dis =camera.get_current_frame()["distance_to_camera"]
            where_are_inf = np.isinf(dis)
            dis[where_are_inf] = 5
            print(type(dis))
            dis= (dis-np.min(dis))/(np.max(dis)-np.min(dis))
            
            # if camera.get_depth():
            #     print(camera.get_depth().shape)
            plt.imsave("D:\MMLM_Robot\Grasp_Nav\code\EmbodiedAI//tests//obs//depth//"+f"frame_{i:03d}.png",dis,cmap="gray")
            #     # print(action)
       
        # # print(my_lidar.get_current_frame())
        # print("left:",imu_sensor_left.get_current_frame())
        # print("right:",imu_sensor_left.get_current_frame())
        # if i>= 700:break
        # if i >= 100:
        i += 1

# with open("map_pos_data.txt","w") as f:
#         for i in range(len(pos_list)):
#             f.write(str(pos_list[i])+"\n")
simulation_app.close()

