from simulator.core.robot import BaseRobot
from simulator.core.config import RobotConfig
from simulator.core.register import registry

@registry.register_robot(name="Stretch")
class Stretch(BaseRobot):
    def __init__(self, robot_config:RobotConfig):
        super.__init__(robot_config)
        self.prim_path = "/World/Robot/Stretch" if self.prim_path==None else robot_config.prim_path
        pass
    
    
    @property
    def robot_name(self) -> str:
        '''
        robot's name, e.g. "Stretch", "Gen3_robotiq85"
        '''
        return "Stretch"
    
    @property
    def robot_type(self) -> str:
        '''
        robot's type, e.g. "Wheel", "Leg", "Arm", "Drone"
        '''
        return "Wheel"
    
    @property
    def observation(self):
        '''
        Get the observation of the robot from the sensor
        '''
        pass
    
    @property
    def state(self):
        '''
        Get the state of the robot, e.g. position, pose
        '''
        pass
    
    @property
    def get_wheel_names(self) -> list[str]:
        '''
        Get the name of the Stretch's wheels
        '''
        return ["joint_left_wheel", "joint_right_wheel"]
    
    @property
    def get_arm_names(self) -> list[str]:
        '''
        Get the name of the Stretch's arm joints
        '''
        return ["joint_lift",
                "joint_arm_l3",
                "joint_arm_l2",
                "joint_arm_l1",
                "joint_arm_l0",
                "joint_wrist_yaw",
                "joint_wrist_pitch",
                "joint_wrist_roll",]
        
    
    def init(self):
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

    def reset(self):
        pass
    
    def obj_grasp(self):
        pass
    
    def obj_release(self):
        pass
    

    # position control
    def move_to_target_velocity(self, target):
        pass
    
    
    
    
    
    
    