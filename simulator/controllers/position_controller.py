from simulator.core.config import ControllerConfig
from simulator.core.register import registry
from simulator.core.controller import BaseController
import numpy as np
import math

@registry.register_controller
class PositionController(BaseController):
    type: str = "position"
    input_limit: str = "default"
    output_limit: str = "default"
    name: str = "position_controller"
    def __init__(self, config:ControllerConfig):
        super().__init__(config)
    

    def get_action(self,str) -> np.ndarray:
        '''
        w: Move forward
        s: Move backward
        a: Turn left
        d: Turn right
        '''
        
        return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    
    def forward_by_mode(self):
        pass
    
    
    def turn_by_mode(self):
        pass
    
    
    def move_by_target(self, target, world, orientation=False):
        """
        Move the robot to a target position and orientation.
        :param target: Target position (x, y) and orientation (w) in radians.
        :param world: Simulation world.
        :param orientation: Whether to adjust the orientation in the process.
        """
        
        pass
    
    
    def move_by_mode(self):
        pass
    
    def trans_pos(self):
        pass
    
    
    def trans2pi(self,num):
        """
        Convert angle difference to the range [-pi, pi].
        :param num: Angle difference.
        :return: Angle within the range [-pi, pi].
        """
        while num < -math.pi or num > math.pi:
            if num < -math.pi:
                num += 2*math.pi
            else:
                num -= 2*math.pi
        return num
    
    
    