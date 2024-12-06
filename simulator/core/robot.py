from abc import ABC, abstractmethod
from simulator.core.config import RobotConfig, SensorConfig, ControllerConfig
import numpy as np
from lazyimport import lazyimport
from simulator.controllers import make_controller
from simulator.sensors import make_sensor

lazyimport(
    globals(), 
    """
    from omni.isaac.core.robots import Robot as OmniBaseRobot
    """
)
class BaseController(ABC):
    def __init__(self, config: dict):
        self.config = config
        self.type = config.type
    @abstractmethod
    def get_action(self) -> np.ndarray:
        pass

class BaseSensor(ABC):
    def __init__(self):
        pass
    @abstractmethod
    def get_observation(self) -> np.ndarray:
        pass

class BaseRobot(ABC):
    def __init__(self, robot_config: RobotConfig):
        self.robot_config = robot_config
        self.prim_path = robot_config.prim_path
        self.usd_path = robot_config.usd_path
        self.name = robot_config.name
        self.isaac_robot = OmniBaseRobot(prim_path=self.prim_path, 
                        name=self.name, 
                        usd_path=self.usd_path, **kwargs)
        
        self.joints = config.joints
        self.joint_names = [joint.name for joint in self.joints]
        
        self.controllers = [make_controller(controller.type, controller) for controller in robot_config.controllers]
        self.sensors = [make_sensor(sensor.type, sensor) for sensor in robot_config.sensors]
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
        pass

