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


@registry.register_controller
class PositionController(BaseController):
    
    def __init__(self, config:ControllerConfig):
        super().__init__(config)
        self.forward_m = config.forward_m if config.forward_m is not None else 0.02
        self.angle_yaw = config.angle_yaw if config.angle_yaw is not None else 0.02
    

    def get_action(self, command: str, robot=None) -> np.ndarray:
        """
        Generate action based on movement command.
        
        Args:
            command: Movement command (w/s/a/d)
            length: Linear displacement (default: 0.02)
            angle_yaw: Angular displacement (default: 0.02 rad)
            
        Returns:
            np.ndarray: Target position and yaw angle
        """
        length = self.forward_m
        angle_yaw = self.angle_yaw
        position, roll, pitch, yaw = self.trans_pos(robot)
        x_dir = np.array([math.cos(yaw), math.sin(yaw), 0])
        
        if command == 'w':
            target_pos = position + length * x_dir
        elif command == 's':
            target_pos = position - length * x_dir
        else:
            target_pos = position
        
        new_yaw = yaw
        if command == 'a':
            new_yaw = (yaw + angle_yaw) % (2 * math.pi)
        elif command == 'd':
            new_yaw = (yaw - angle_yaw) % (2 * math.pi)
        
        euler = quaternion_from_euler(roll, pitch, new_yaw)
        return target_pos, euler
    
    
    def trans_pos(self, robot)-> tuple:
        """
        Get robot's current position and orientation.
        
        Args:
            robot: Robot instance
            
        Returns:
            tuple: (position, roll, pitch, yaw)
        """
        robot_pos = Robot(robot)
        position, quaternion  = robot_pos.get_world_pose()
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
        return 1