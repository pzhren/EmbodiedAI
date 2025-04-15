from simulator.core.robot import BaseRobot
from simulator.core.config import RobotConfig
from simulator.core.register import registry


@registry.register_robot
class Stretch(BaseRobot):
    def __init__(self, robot_config:RobotConfig):
        super().__init__(robot_config)

        self.prim_path = "/World/Robot/Stretch" if self.prim_path==None else robot_config.prim_path
        self.use_position = robot_config.use_position if robot_config.use_position is not None else False
        self.grasped_object = {}
        self.plan_length = []
        pass
    
    
    @property
    def robot_type(self) -> str:
        '''
        robot's type, e.g. "Wheel", "Leg", "Arm", "Drone"
        '''
        return "Wheel"


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
                "joint_wrist_roll",
                "joint_gripper_finger_left",
                "joint_gripper_finger_right"]
    
    @property
    def lift_arm(self):
        return self.get_arm_names()[0]
    
    @property
    def grasp_left_arm(self):
        return self.get_arm_names()[-2]
    
    @property
    def grasp_right_arm(self):
        return self.get_arm_names()[-1]
    
    @property
    def grasp_arm0(self):
        return self.get_arm_names()[4]

    @property
    def grasp_arm1(self):
        return self.get_arm_names()[3]
    
    @property
    def grasp_arm2(self):
        return self.get_arm_names()[2]

    @property
    def grasp_arm3(self):
        return self.get_arm_names()[1]
        
    
    def init(self):
        super().init()
        pass
    
    
    def apply_action(self, action_instruct, world):
        id = 0
        for controller in self.controllers:
            dim = controller.action_dim
            # command = controller.get_action(action_instruct[id:id+dim], robot=self.prim_path)
            # id+=dim
            if len(action_instruct[id:id+dim]) != dim:
                action = [0 for _ in range(dim)]
            else:
                action = action_instruct[id:id+dim]
            
            return_dict = controller.step(self, world, action, self.grasped_object)
            if return_dict is not None and isinstance(return_dict, dict):
                for k,v in return_dict.items():
                    setattr(self, k, v)
            id+=dim 
        # if self.use_position:
        #     self.Xform.set_world_pose(position = command[0],orientation = command[1])
        # else:
        #     self.isaac_robot.apply_action(command)

    def reset(self):
        self.grasped_object = {}
    
    def get_path_length(self):
        return sum(self.plan_length)
    