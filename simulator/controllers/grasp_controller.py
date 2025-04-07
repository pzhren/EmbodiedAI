from simulator.core.config import ControllerConfig
from simulator.core.register import registry
from simulator.core.controller import BaseController
from simulator.core.utils.grasp_utils import transform_to_base, transform_to_world, rpy2R
from transformations import euler_from_quaternion, quaternion_from_euler
from lazyimport import lazyimport

lazyimport(
    globals(),
    """
    from omni.isaac.core.robots import Robot
    from omni.isaac.core.utils.types import ArticulationAction
    """
)

import math
import numpy as np

@registry.register_controller
class StretchGraspController(BaseController):
    def __init__(self, config: ControllerConfig):
        super().__init__(config)
        self.arm_length = 0.52
        self.lift_height = 0
        self.grasp_angle = 0
        self.is_grasping = False
        self.obj_prim = None

        self.arm_speed = 0.05
        self.lift_speed = 0.05
        self.gripper = 0.2
    
    def reset(self):
        self.obj_prim = None
        
    def get_action(self, command: List, robot=None) -> np.ndarray:
        if command != None:
            if command[0] == "grasp":
                self.obj_prim = command[1]
                target_obj = self.obj_prim.get_world_pose()

        return command
    
    def step(self, robot, world, command: str):
        command = self.get_action(command, robot)
    

    def grasp_by_target(self, robot, target, world):
        """
        Move the robotic arm to the target position and grasp the object.
        :param target: Target position [length, height]
        :param world: Simulation world object
        """
        lift_dof = robot.get_dof_index(self.lift_arm)
        grasp_left_dof = robot.get_dof_index(self.grasp_left_arm)
        grasp_right_dof = robot.get_dof_index(self.grasp_right_arm)
        grasp_arm0_dof = robot.get_dof_index(self.grasp_arm0)
        grasp_arm1_dof = robot.get_dof_index(self.grasp_arm1)
        grasp_arm2_dof = robot.get_dof_index(self.grasp_arm2)
        grasp_arm3_dof = robot.get_dof_index(self.grasp_arm3)

        time_step = int(target[1]/self.lift_speed)
        # lift the robotic arm to the specified height
        for i in range(time_step):
            action = ArticulationAction(joint_positions=np.array([self.lift_speed * i]), joint_indices=np.array([lift_dof]))
            robot.apply_action(action)
            world.step(render=True)  
        # self.dc.set_dof_position_target(self.dof_lift, target[1]) # Lift the robotic arm to the specified height
        self.lift = target[1] # Update lift height
        
        for i in range(0,50):
            action = ArticulationAction(joint_positions=np.array([i*self.gripper,i*self.gripper]), joint_indices=np.array([grasp_left_dof,grasp_right_dof]))
            robot.apply_action(action)
            world.step(render=True)


        time_step = int(target[0]/4 / self.arm_speed)
        dof_arms = [grasp_arm0_dof,grasp_arm1_dof,grasp_arm2_dof,grasp_arm3_dof]
        for i in range(len(dof_arms)):
            for j in range(time_step):
                action = ArticulationAction(joint_positions=np.array([self.arm_speed * j]), joint_indices=np.array([dof_arms[i]]))
                robot.apply_action(action)
                world.step(render=True)

        # self.dc.set_dof_position_target(self.dof_arm1,target[0]/4)
        # self.dc.set_dof_position_target(self.dof_arm2,target[0]/4)
        # self.dc.set_dof_position_target(self.dof_arm3,target[0]/4)
        # self.dc.set_dof_position_target(self.dof_arm4,target[0]/4)
        self.arm = target[0] # Update arm length


        for i in range(50,0,-1*self.gripper):
            action = ArticulationAction(joint_positions=np.array([i * 0.1,i * 0.1]), joint_indices=np.array([grasp_left_dof,grasp_right_dof]))
            robot.apply_action(action)
            world.step(render=True)
        
        self.is_grasping = True
        
        tmp_length = target[0] / 4
        while tmp_length > 0:
            action= ArticulationAction(joint_positions=np.array([tmp_length,tmp_length,tmp_length,tmp_length]), joint_indices=np.array([grasp_arm0_dof, grasp_arm1_dof, grasp_arm2_dof, grasp_arm3_dof]))
            robot.apply_action(action)
            world.step(render=True)
            tmp_length -= self.arm_speed
            if tmp_length <= 0:
                tmp_length = 0
            self.arm = tmp_length

            # Attach the object to the robotic arm's gripper
            self.d = (-0.02, -0.4 - self.arm, 0.06 + self.lift)
            cur_position,roll,pitch,yaw = self.trans_pos()
            
            R = self.rpy2R([roll,pitch,yaw]) # Obtain the rotation matrix between the world and vehicle coordinates
            R = np.linalg.inv(R)
            # Multiply d by the rotation matrix and add the current position to get the world coordinates of the gripper
            self.d = np.dot(self.d,R)
            grasp_position = cur_position + self.d
            self.obj_prim.set_world_pose(position=np.array(grasp_position))
            world.step(render=True)
    

    def release_by_target(self, robot, target, world):
        '''
        Move the robotic arm to the target position and release the object.
        :param target: Target position [length, height]
        :param world: Simulation world object
        '''
        tmp_height = target[1]
        while abs(tmp_height - self.lift) > 0.01 and self.lift <= 1:
            action = ArticulationAction(joint_positions=np.array([self.lift + self.lift_speed * (tmp_height - self.lift)/abs(tmp_height - self.lift)]), joint_indices=np.array([lift_dof]))
            robot.apply_action(action)
            world.step(render=True)
            
            self.lift = self.lift + self.lift_speed * (tmp_height - self.lift)/abs(tmp_height - self.lift)

            self.d = (-0.02, -0.4 - self.arm, 0.06 + self.lift)
            cur_position,roll,pitch,yaw = self.trans_pos(robot)
            
            R = self.rpy2R([roll,pitch,yaw])
            R = np.linalg.inv(R)
            self.d = np.dot(self.d,R)
            grasp_position = cur_position + self.d
            self.obj_prim.set_world_pose(position=np.array(grasp_position))
            world.step(render=True)    


        tmp_length = 0
        while tmp_length < target[0] / 4:
            action= ArticulationAction(joint_positions=np.array([tmp_length,tmp_length,tmp_length,tmp_length]), joint_indices=np.array([grasp_arm0_dof, grasp_arm1_dof, grasp_arm2_dof, grasp_arm3_dof]))
            robot.apply_action(action)
            world.step(render=True)
            
            tmp_length += self.arm_speed
            world.step(render=True)
            self.arm = tmp_length*4

            self.d = (-0.02, -0.4 - self.arm, 0.06 + self.lift)
            cur_position,roll,pitch,yaw = self.trans_pos()
            
            R = self.rpy2R([roll,pitch,yaw])
            R = np.linalg.inv(R)
            self.d = np.dot(self.d,R)
            grasp_position = cur_position + self.d
            self.obj_prim.set_world_pose(position=np.array(grasp_position))
            world.step(render=True)    


        for i in range(0,101):
            action = ArticulationAction(joint_positions=np.array([i * 0.1,i * 0.1]), joint_indices=np.array([grasp_left_dof,grasp_right_dof]))
            robot.apply_action(action)
            world.step(render=True)

        self.is_grasping = False  # Set the grasping status to false 

        tmp_length = target[0] / 4
        while tmp_length > 0:
            action= ArticulationAction(joint_positions=np.array([tmp_length,tmp_length,tmp_length,tmp_length]), joint_indices=np.array([grasp_arm0_dof, grasp_arm1_dof, grasp_arm2_dof, grasp_arm3_dof]))
            robot.apply_action(action)
            world.step(render=True)
            tmp_length -= self.arm_speed
            if tmp_length <= 0:
                tmp_length = 0
            self.arm = tmp_length # 更新arm长度
            
        # self.dc.set_dof_position_target(self.dof_arm4,0)
        # self.dc.set_dof_position_target(self.dof_arm3,0)
        # self.dc.set_dof_position_target(self.dof_arm2,0)
        # self.dc.set_dof_position_target(self.dof_arm1,0)
        # self.arm = 0 # Update arm length
        world.step(render=True)

        for i in range(100,0,-1):
            action = ArticulationAction(joint_positions=np.array([i * 0.1, i * 0.1]), joint_indices=np.array([grasp_left_dof,grasp_right_dof]))
            robot.apply_action(action)
            world.step(render=True)
        pass


    def trans_pos(self, robot=None):
        """
        Get the position and orientation of the Stretch robot's base link in the world coordinate system.
        :return: Position (x, y, z), Roll, Pitch, Yaw
        """
        # stretch_baselink_pos = Robot(self.robot)
        position, quaternion  = robot.get_world_pose()
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        return position, roll, pitch , yaw


    @property
    def action_dim(self) -> int:
        return 2

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