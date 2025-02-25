from simulator.core.robot import BaseRobot
from simulator.core.config import RobotConfig
from simulator.core.register import registry


@registry.register_robot
class Stretch(BaseRobot):
    def __init__(self, robot_config:RobotConfig):
        super().__init__(robot_config)

        self.prim_path = "/World/Robot/Stretch" if self.prim_path==None else robot_config.prim_path
        self.use_position = robot_config.use_position if robot_config.use_position is not None else False
        pass
    
    
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
    def get_world_pose(self):
        '''
        Get the state of the robot, position, pose
        '''
        return self.isaac_robot.get_world_pose()
    
    
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
        super().init()
        pass
    
    
    def apply_action(self, action_instruct):
        id = 0
        for controller in self.controllers:
            dim = controller.action_dim
            command = controller.get_action(action_instruct[id:id+dim], robot=self.prim_path)
            id+=dim 
        if self.use_position:
            self.Xform.set_world_pose(position = command[0],orientation = command[1])
        else:
            self.isaac_robot.apply_action(command)

    def reset(self):
        pass


    def obj_grasp(self):
        pass
    
    
    def obj_release(self):
        pass
    

    def move_to_target(self, target):
        self.controllers.move_to_target(target)
    
    
    