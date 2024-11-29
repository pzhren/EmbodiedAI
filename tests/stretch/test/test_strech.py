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

my_world = World(stage_units_in_meters=1.0)
assets_root_path = get_assets_root_path()
my_world.scene.add_default_ground_plane()

stretch_path ="E:\MMLM_Robot\Grasp_Nav\code\EmbodiedAI\\tests\stretch\model\stretch.usd" 

stretch=prim_utils.create_prim(
    prim_type="Xform",
    prim_path = "/World/Robot",
    usd_path=stretch_path,
    # translation=np.array([0.0, 0.0, -0.01]),
    # usd_path="D:\MMLM_Robot\Grasp_Nav\code//reconstruct_scene\drjohn.usd",
)

stretch = WheeledRobot(
        prim_path="/World/Robot",
        name="stretch",
        wheel_dof_names=["link_left_wheel", "link_right_wheel"],
        create_robot=True,
        usd_path=stretch_path,
        position=np.array([0.0, 0.0, 0.0]),
    )

my_controller = DifferentialController(name="simple_control", wheel_radius=10.16, wheel_base=34)
camera = Camera("/World/Robot/camera_color_frame/rgb_camera",
                resolution=(1280,640),
                # orientation = np.array([1, 0, 0, 0]),
                translation =np.array([0.0, 0.0, 0.0]),)
# depth_camera = Camera("/World/Robot/camera_depth_frame",resolution=(1280,640))
imu_sensor = my_world.scene.add(
    IMUSensor(
        prim_path="/World/Robot/base_imu/imu_sensor",
        name="imu",
        frequency=60,
        translation=np.array([0, 0, 0]),
    )
)

camera.initialize()
camera.add_motion_vectors_to_frame()
camera.add_distance_to_camera_to_frame()

reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            # my_controller.reset()
            reset_needed = False
        stretch.apply_wheel_actions(my_controller.forward(command=[0.2, -np.pi/2]))
        print("imu:",imu_sensor.get_current_frame())
        if camera.get_rgb().any():
            print(type(camera.get_rgb()))
            print(camera.get_rgb().shape)
            plt.imsave("E:\MMLM_Robot\Grasp_Nav\code\EmbodiedAI//tests//obs//rgb//"+"frame_str.png",camera.get_rgb())
            dis =camera.get_current_frame()["distance_to_camera"]
            where_are_inf = np.isinf(dis)
            dis[where_are_inf] = 5
            print(type(dis))
            dis= (dis-np.min(dis))/(np.max(dis)-np.min(dis))
            
            # if camera.get_depth():
            #     print(camera.get_depth().shape)
            plt.imsave("E:\MMLM_Robot\Grasp_Nav\code\EmbodiedAI//tests//obs//depth//"+"frame_str.png",dis,cmap="gray")

simulation_app.close()
