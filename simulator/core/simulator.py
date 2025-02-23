import isaacsim
import carb
from omni.isaac.kit import SimulationApp
from typing import Any, Callable, Dict, List, Optional
from simulator.core.config import SimulatorConfig, PrimConfig
from simulator.utils.log_utils import create_module_log
from simulator.core.robot import BaseRobot
from simulator.core.scene import BaseScene


from omni.isaac.dynamic_control import _dynamic_control
from transformations import euler_from_quaternion,quaternion_from_euler
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.robots import Robot
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from keyboard_interact import KeyboardController



from lazyimport import lazyimport
lazyimport(globals(), """
    from omni.isaac.core import World
    import omni.isaac.core.utils.prims as prim_utils
    from omni.isaac.nucleus import get_assets_root_path
  """
)

def launch_app(app_config:dict, webrtc=False, native=False):
    """
    Launch the isaac sim app.
    """
    simulation_app = SimulationApp(app_config)
        
    if webrtc:
        from omni.isaac.core.utils.extensions import enable_extension  

        simulation_app.set_setting('/app/window/drawMouse', True)
        simulation_app.set_setting('/app/livestream/proto', 'ws')
        simulation_app.set_setting('/app/livestream/websocket/framerate_limit', 60)
        simulation_app.set_setting('/ngx/enabled', False)
        enable_extension('omni.services.streamclient.webrtc')

    elif native:
        from omni.isaac.core.utils.extensions import enable_extension  

        simulation_app.set_setting("/app/window/drawMouse", True)
        simulation_app.set_setting("/app/livestream/proto", "ws")
        simulation_app.set_setting("/app/livestream/websocket/framerate_limit", 120)
        simulation_app.set_setting("/ngx/enabled", False)
        enable_extension("omni.kit.livestream.native")

    return simulation_app

class Simulator():
    def __init__(self, config:SimulatorConfig, webrtc=False, native=False):
        
        self.phy_dt = 1./config.physics_frequency 
        self.render_dt = 1./config.render_frequency
        self.headless = config.headless
        self.height = config.height
        self.width = config.width
        self.hide_ui = config.hide_ui
        self.device = config.device
        self.app_config = {
            'headless': self.headless, 
            'height': self.height, 
            'width': self.width, 
            'physics_gpu': self.device,
            'active_gpu': self.device,
            }
        try:
            self._simulation_app = launch_app(self.app_config, webrtc=webrtc, native=native)
        except Exception as e:
            log.error("Can not lanuch isaac sim app.")
            raise e

        self._world = World(physics_dt=self.phy_dt, rendering_dt=self.render_dt, stage_units_in_meters=1.0)
        self._scene = self._world.scene
        self._stage = self._world.stage
        self._resource_path = "./resource"
        self._warm_up()
        self.is_playing = self._world.is_playing()

    def _warm_up(self, steps=20, render=True):
        """
        Warm up the simulator.
        """
        for _ in range(steps):
            self._world.step(render=render)
    
    def add_tasks(self, tasks):
        for config in configs:
            task = create_task(config, self._scene)
            self._world.add_task(task)

        self._world.reset()
        self._warm_up()
    
    def import_robot(self, robot):
        assert self.is_playing==False
        assert isinstance(robot, BaseRobot)
        # prim_utils
        prim_utils.create_prim(
            prim_type = "Xform",
            prim_path = robot.prim_path,
            usd_path = robot.usd_path,
            translation = robot.position,
            orientation = robot.orientation,
            scale = robot.scale,
            # orientation=[0.70711,0.0,0.0,-0.70711]
            )
        pass

    # def import_sensor(self, sensor):
    #     assert self._world.is_playing==False
    #     assert isinstance(sensor, BaseSensor)
    #     if sensor.type=="vision":
    #         prim_utils.create_prim(
    #             prim_type = "Camera",
    #             prim_path = sensor.prim_path,
    #             position = sensor.postion,
    #             orientation = sensor.orientation,
    #         )

    #     return sensor

    def import_scene(self, scene):
        assert self.is_playing==False
        assert isinstance(scene, BaseScene)

        if scene._use_floor_plane:
            assets_root_path = get_assets_root_path()
            self._world.scene.add_default_ground_plane()
            if not scene._floor_plane_visible:
                prim_utils.set_visibility(prim_path="/World/defaultGroundPlane", visible=False)
        
        if scene._use_sky_box:
            prim_utils.create_prim(
                prim_path="/World/defaultSky",
                prim_type = "DomeLight",
                attributes={
                    "inputs:intensity": 1500,
                    # "inputs:radius":1.0,
                    "inputs:texture:file": f"{self._resource_path}/background/sky/sky.jpg"
                }
            )
        
        for k,v in scene.scene_prim_dict.items():
            pass
    def play(self):
        return self._world.play()
    
    def position_control(self, target, world,orientation=True):
        """
        Move the robot to a target position and orientation.
        :param target: Target position (x, y) and orientation (w) in radians.
        :param world: Simulation world.
        :param orientation: Whether to adjust the final orientation.
        """
        # get current position and yaw
        pos, roll, pitch, yaw = self.trans_pos()
        x1, y1, _ = pos
        x2, y2, w2 = target
        angle_to_target = math.atan2(y2 - y1, x2 - x1)
        angle_diff = self.trans2pi(angle_to_target - yaw)
        
        # Turn
        while abs(angle_diff) > self.error_ang:
            euler2q = quaternion_from_euler(roll, pitch, yaw + self.angular_velocity * angle_diff / abs(angle_diff) )
            self.prim.set_world_pose(orientation=np.array(euler2q),position=np.array([x1, y1, self.z]))
            # get process image
            _, _, _, yaw = self.trans_pos()
            angle_diff = self.trans2pi(angle_to_target - yaw)
            world.step(render=True)

        
        # Move 
        while math.sqrt((x2 - x1)**2 + (y2 - y1)**2) > self.error_dis: 
            dx = x2 - x1
            dy = y2 - y1
            self.prim.set_world_pose(position=np.array([x1 + self.linear_velocity*dx/math.sqrt(dx**2 + dy**2),y1 + self.linear_velocity*dy/math.sqrt(dx**2 + dy**2), self.z]))
            world.step(render=True)
        
            pos, roll, pitch, yaw = self.trans_pos()
            x1, y1 = pos[0], pos[1]


        # Final turn to target orientation
        if orientation:
            final_angle_diff = w2 - yaw
            final_angle_diff = self.trans2pi(final_angle_diff)
    
            while abs(final_angle_diff) >self.error_ang:
                euler2q = quaternion_from_euler(roll, pitch, yaw+self.angular_velocity*final_angle_diff/abs(final_angle_diff))
                self.prim.set_world_pose(orientation=np.array(euler2q))
                
                pos, roll, pitch, yaw = self.trans_pos()
                final_angle_diff = self.trans2pi(w2 - yaw)
                world.step(render=True)
                
        print("current_pos:",pos)
    
    
    def step(self, render:bool=True) -> dict[str, Any]:
        return self._world.step(render=render)

    def reset(self):
        self._world.reset()

    def close(self):
        self._simulation_app.close()

    @property
    def current_tasks(self):
        return self._world._current_tasks

    @property
    def is_running(self) -> bool:
        return self._simulation_app.is_running()

   
    
  