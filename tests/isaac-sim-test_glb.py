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
# from omni.isaac.core.utils.prims import create_prim
from omni.isaac.sensor import Camera,RotatingLidarPhysX,IMUSensor
from omni.isaac.core.scenes import Scene as OmniBaseScene
import matplotlib.pyplot as plt
import omni.kit.viewport.utility as vu 
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

construct_scene=prim_utils.create_prim(
    prim_type="Xform",
    prim_path = "/World/Scene",
    usd_path="E:\MMLM_Robot\Grasp_Nav\code\\102343992.glb",
    # translation=np.array([0.0, 0.0, -0.01]),
    # usd_path="D:\MMLM_Robot\Grasp_Nav\code//reconstruct_scene\drjohn.usd",
)