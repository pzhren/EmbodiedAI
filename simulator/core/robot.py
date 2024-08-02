from abc import ABC
from omni.isaac.core.robos import Robot as OmniBaseRobot
from simulator.core.config import RobotConfig

class BaseController(ABC):
  pass

class BaseSensor(ABC):
    def __init__(self):
        pass

class BaseRobot(OmniBaseRobot, ABC):
    def __init__(self, robot_config: RobotConfig):
        self.robot_config = robot_config
        super().__init__(prim_path=prim_path, 
                        name=name, 
                        usd_path=usd_path, **kwargs)
        
        self.joints = config.joints
        self.joint_names = [joint.name for joint in self.joints]
    
    def apply_action(self, action):
        return super.apply_action(action)

    def get_joints(self) -> list[Joint]:
        return self.get_components_by_type(Joint)
    
    def get_joint_names(self) -> list[str]:
        return [joint.name for joint in self.joints]
    
    def get_joint_positions(self) -> np.ndarray:
        return np.array([joint.get_position() for joint in self.joints])



