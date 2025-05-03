import os
os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"


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
from omni.isaac.core.scenes import Scene as OmniBaseScene
import matplotlib.pyplot as plt
import omni.kit.viewport.utility as vu 


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

robot = Robot("/World/Jetbot")
robot.set_local_scale(np.array([2.2,2.2,2.2]))

Plane = RigidPrim("/World/defaultGroundPlane")
Plane.set_visibility(visible=False)
Plane.disable_rigid_body_physics()

my_controller = DifferentialController(name="simple_control", wheel_radius=0.03, wheel_base=0.1125)
my_world.reset()
i = 0
reset_needed = False

camera.initialize()
camera.add_motion_vectors_to_frame()
camera.add_distance_to_camera_to_frame()

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
        if i >= 0 and i < 300:
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
    
        if camera.get_rgb().any():
            print(type(camera.get_rgb()))
            print(camera.get_rgb().shape)
            dis =camera.get_current_frame()["distance_to_camera"]
            where_are_inf = np.isinf(dis)
            dis[where_are_inf] = 5
            print(type(dis))
            
        i += 1

simulation_app.close()

