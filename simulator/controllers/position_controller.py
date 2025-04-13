from simulator.core.config import ControllerConfig
from simulator.core.register import registry
from simulator.core.controller import BaseController

from transformations import euler_from_quaternion, quaternion_from_euler
from lazyimport import lazyimport

lazyimport(
    globals(),
    """
    from omni.isaac.core.robots import Robot
    """
)

import math
import numpy as np
from gym import spaces

@registry.register_controller
class PositionController(BaseController):
    
    def __init__(self, config:ControllerConfig):
        super().__init__(config)
        self._max_forward_m = config.forward_m if config.forward_m is not None else 0.5
        self._max_angle_yaw = config.angle_yaw if config.angle_yaw is not None else np.pi/2.
        

    def get_action(self, command: list, robot=None) -> np.ndarray:
        """
        Generate action based on movement command.
        
        Args:
            command: Movement command (w/s/a/d)
            length: Linear displacement (default: 0.02)
            angle_yaw: Angular displacement (default: 0.02 rad)
            
        Returns:
            np.ndarray: Target position and yaw angle
        """
        
        
        position, roll, pitch, yaw = self.trans_pos(robot)
        x_dir = np.array([math.cos(yaw), math.sin(yaw), 0])
        
        if command[0] == 'w':
            length = min(self._max_forward_m, command[1]) # in meters
            target_pos = position + length * x_dir

        elif command[0] == 's':
            length = min(self._max_forward_m, command[1]) # in meters
            target_pos = position - length * x_dir
        else:
            target_pos = position
        
        new_yaw = yaw
        if command[0] == 'a':
            angle_yaw = min(self._max_angle_yaw,command[1]) # in radians
            new_yaw = (yaw + angle_yaw) % (2 * math.pi)
        elif command[0] == 'd':
            angle_yaw = min(self._max_angle_yaw,command[1])
            new_yaw = (yaw - angle_yaw) % (2 * math.pi)
        
        euler = quaternion_from_euler(roll, pitch, new_yaw)
        return target_pos, euler
    
    def step(self, robot, world, command, grasped_object):
        target_pos, euler = self.get_action(command, robot)
        if command[0]=="w":
            robot.plan_length.append(command[1])
        robot.Xform.set_world_pose(position = target_pos, orientation = euler)
        if grasped_object is not None:
            for k,v in grasped_object:
                target_pos, euler = self.get_action(command, v)
                v.set_world_pose(position = target_pos, orientation = euler)
        return 1
    
    def trans_pos(self, robot)-> tuple:
        """
        Get robot's current position and orientation.
        
        Args:
            robot: Robot instance
            
        Returns:
            tuple: (position, roll, pitch, yaw)
        """
        position, quaternion  = robot.get_world_pose(), robot.get_world_orientation()
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        return position, roll, pitch, yaw 
    
    
    @staticmethod
    def normalize_angle(angle: float)-> float:
        """
        Normalize angle to [-π, π] range.
        
        Args:
            angle: Input angle in radians
            
        Returns:
            float: Normalized angle
        """
        while angle < -math.pi or angle > math.pi:
            if angle < -math.pi:
                angle += 2*math.pi
            else:
                angle -= 2*math.pi
        return angle

    @property
    def action_dim(self) -> int:
        """
        Get action dimension.
        
        Returns:
            int: Action dimension
        """
        return 2

    @property
    def action_space(self) -> spaces.Tuple:
        """
        Get action space.
        
        Returns:
            spaces.Box: Action space
        """
        return spaces.Tuple((
            spaces.Discrete(3),  # 动作类型索引
            spaces.Box(low=0, high=self._max_forward_m, shape=(1,), dtype=np.float32)  # 参数值
        ))

    