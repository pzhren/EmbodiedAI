from abc import ABC, abstractmethod
from simulator.core.config import RobotConfig
import numpy as np
from lazyimport import lazyimport
from simulator.controllers import make_controller
from simulator.sensors import make_sensor

lazyimport(
    globals(), 
    """
    from omni.isaac.core.robots import Robot as OmniBaseRobot
    from omni.isaac.core.prims import XFormPrim
    """
)

class BaseRobot(ABC):
    def __init__(self, robot_config: RobotConfig):
        self.robot_config = robot_config
        self.prim_path = robot_config.prim_path
        self.usd_path = robot_config.usd_path
        self.name = robot_config.name
        self.position = robot_config.position
        self.orientation = robot_config.orientation
        self.scale = robot_config.scale
        self.init_joints = robot_config.init_joints
        self.action_dim = 0
        if robot_config.controllers is not None:
            # self.controllers = []
            self.controllers = [make_controller(controller.type, controller) for controller in robot_config.controllers if controller is not None]
            for controller in self.controllers:
                self.action_dim += controller.action_dim
        if robot_config.sensors is not None:
            # self.sensors = []
            self.sensors = [make_sensor(sensor.type, sensor) for sensor in robot_config.sensors if sensor is not None]
    def apply_action(self, action):
        return self.isaac_robot.apply_action(action)
    
    def get_joint_names(self) -> list[str]:
        return [joint.name for joint in self.joints]
    
    def get_joint_positions(self) -> np.ndarray:
        return np.array([joint.get_position() for joint in self.joints])

    def init(self):
        """
        Initialize the robot
        """
        self.isaac_robot = OmniBaseRobot(prim_path=self.prim_path, name=self.name)
        self.Xform = XFormPrim(self.prim_path)
        pass

