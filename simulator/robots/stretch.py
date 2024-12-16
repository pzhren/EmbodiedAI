from simulator.core.robot import BaseRobot
from simulator.core.config import RobotConfig
from simulator.core.register import registry

@registry.register_robot(name="Stretch")
class Gen3_robotiq85(BaseRobot):
    def __init__(self, robot_config:RobotConfig):
        super.__init__(robot_config)
        self.prim_path = "/World/Robot/Stretch" if self.prim_path==None else robot_config.prim_path
        pass
    
    
    @property
    def robot_name(self) -> str:
        return "Stretch"
    
    @property
    def robot_type(self) -> str:
        return "Wheel"
    
    @property
    def observation(self):
        pass
    
    @property
    def state(self):
        pass
    
    
    def apply_action(self, action_instuct):
        if action_instuct == "Forward":
            pass
        elif action_instuct == "Backward":
            pass
        elif action_instuct == "Turn left":
            pass
        elif action_instuct == "Turn right":
            pass
        elif action_instuct == "Stop":
            pass
        else:
            pass

    def get_joint_names(self) -> list[str]:
        pass

    def init(self):
        pass
    
    def reset(self):
        pass
    
    def obj_grasp(self):
        pass
    
    def obj_release(self):
        pass
    
    # friction control
    def move_to_target_velocity(self, target):
        pass
    
    # position control
    def move_to_target_velocity(self, target):
        pass
    
    
    
    
    
    
    