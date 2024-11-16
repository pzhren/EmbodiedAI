import numpy as np
from scipy.spatial.transform import Rotation as R
from omni.isaac.dynamic_control import _dynamic_control
import numpy as np
import math
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.robots import Robot
from transformations import euler_from_quaternion,quaternion_from_euler


# 机械臂抓取控制部分
class BaseGrasp:
    def __init__(self,arm1,arm2,arm3,arm4,lift,grasp,obj_prim,robot):
        self.arm = 0 # 机械臂长度
        self.lift = 0 # 机械臂高度
        self.grasp = 0 # 机械臂夹角
        self.d = (-0.02, -0.4 - self.arm, 0.06 + self.lift) # 机械臂末端到机器车中心的偏移，在机器车坐标系下

        self.dc = _dynamic_control.acquire_dynamic_control_interface()

        # 定义lift关节
        self.articulation_lift = self.dc.get_articulation(lift)
        self.dof_lift = self.dc.find_articulation_dof(self.articulation_lift, "joint_lift")

        # 定义arm关节
        self.articulation_arm1 = self.dc.get_articulation(arm1)
        self.dof_arm1 = self.dc.find_articulation_dof(self.articulation_arm1, "joint_arm_l0")

        self.articulation_arm2 = self.dc.get_articulation(arm2)
        self.dof_arm2 = self.dc.find_articulation_dof(self.articulation_arm2, "joint_arm_l1")

        self.articulation_arm3 = self.dc.get_articulation(arm3)
        self.dof_arm3 = self.dc.find_articulation_dof(self.articulation_arm3, "joint_arm_l2")

        self.articulation_arm4 = self.dc.get_articulation(arm4)
        self.dof_arm4 = self.dc.find_articulation_dof(self.articulation_arm4, "joint_arm_l3")


        # 定义grasp关节
        self.articulation_grasp_left = self.dc.get_articulation(grasp)
        self.dof_grasp_left = self.dc.find_articulation_dof(self.articulation_grasp_left, "joint_gripper_finger_left")

        self.articulation_grasp_right = self.dc.get_articulation(grasp)
        self.dof_grasp_right = self.dc.find_articulation_dof(self.articulation_grasp_right, "joint_gripper_finger_right")

        # 是否抓取状态
        self.is_grasping = False
        # 被抓物体prim路径
        self.obj_prim = obj_prim
        self.robot = robot

    def transform_to_world(self, pos, orient):
        """
        将机器车坐标系下的三维坐标转换到世界坐标系下的三维坐标
        :param pos: 机器车坐标系下的三维坐标 (x, y, z)
        :param orient: 机器车相对于世界坐标系的旋转四元数 (w, x, y, z)
        :return: 世界坐标系下的三维坐标 (x, y, z)
        """
        
        rotation = R.from_quat(orient) # 旋转四元数转换为旋转矩阵
        print("rotation",rotation)
        rotation_matrix = rotation.as_matrix() 
        print("rotation_matrix",rotation_matrix)

        world_pos = np.dot(rotation_matrix, self.d) + pos

        return world_pos

    def transform_to_base(self, pos, orient, target):
        """
        由目标位置计算机械臂参数（要求首先移动到正确位置）
        :param pos: 机器车坐标系下的坐标 (x, y, z)
        :param orient: 机器车相对于世界坐标系的旋转四元数 (w, x, y, z)
        :param target: 目标在世界坐标系下的位置 (x, y, z)
        """

        arm_length = 0.52
        rotation = R.from_quat(orient)
        rotation_matrix = rotation.as_matrix() 
        pos_transformed = np.dot(rotation_matrix.T, (target - pos))
        length = abs(-0.4 - pos_transformed[1])
        if length > arm_length:
            length = arm_length
        arg4grasp = [length,abs(pos_transformed[2] - 0.06)]
        return arg4grasp



    def rpy2R(self,rpy): 
        '''
        将stretch机器自身的roll pitch yaw转换为旋转矩阵
        :param rpy: [r,p,y] 单位rad
        '''
        rot_x = np.array([[1, 0, 0],
                        [0, math.cos(rpy[0]), -math.sin(rpy[0])],
                        [0, math.sin(rpy[0]), math.cos(rpy[0])]])
        rot_y = np.array([[math.cos(rpy[1]), 0, math.sin(rpy[1])],
                        [0, 1, 0],
                        [-math.sin(rpy[1]), 0, math.cos(rpy[1])]])
        rot_z = np.array([[math.cos(rpy[2]), -math.sin(rpy[2]), 0],
                        [math.sin(rpy[2]), math.cos(rpy[2]), 0],
                        [0, 0, 1]])
        R = np.dot(rot_z, np.dot(rot_y, rot_x))
        return R



    def grasp_by_target(self, target,world):
        '''
        把机械臂移动到目标位置进行夹取
        '''

        self.dc.set_dof_position_target(self.dof_lift, target[1]) # 把机械臂提升到指定高度
        self.lift = target[1] # 更新lift高度
        
        for i in range(0,101):
            self.dc.set_dof_position_target(self.dof_grasp_left,i * 0.1) # 张开夹爪
            self.dc.set_dof_position_target(self.dof_grasp_right,i * 0.1) # 张开夹爪
            world.step(render=True)


        self.dc.set_dof_position_target(self.dof_arm1,target[0]/4)
        self.dc.set_dof_position_target(self.dof_arm2,target[0]/4)
        self.dc.set_dof_position_target(self.dof_arm3,target[0]/4)
        self.dc.set_dof_position_target(self.dof_arm4,target[0]/4)
        self.arm = target[0] # 更新arm长度


        for i in range(100,0,-1):
            self.dc.set_dof_position_target(self.dof_grasp_left,i * 0.1) # 关闭夹爪
            self.dc.set_dof_position_target(self.dof_grasp_right,i * 0.1) # 关闭夹爪
            world.step(render=True)
        
        self.is_grasping = True
        

        # 设置机械臂缩回来的速度为0.01，每次缩回0.01来是物体跟随机械臂的运动
        tmp_length = target[0] / 4
        while tmp_length > 0:
            self.dc.set_dof_position_target(self.dof_arm1,tmp_length)
            self.dc.set_dof_position_target(self.dof_arm2,tmp_length)
            self.dc.set_dof_position_target(self.dof_arm3,tmp_length)
            self.dc.set_dof_position_target(self.dof_arm4,tmp_length)
            tmp_length -= 0.01
            if tmp_length <= 0:
                tmp_length = 0
            self.arm = tmp_length

            # 吸附物体到机械臂的夹爪上
            self.d = (-0.02, -0.4 - self.arm, 0.06 + self.lift)
            cur_position,roll,pitch,yaw = self.trans_pos()
            
            R = self.rpy2R([roll,pitch,yaw]) # 求取世界坐标和机器坐标的旋转矩阵
            R = np.linalg.inv(R)
            self.d = np.dot(self.d,R) # d乘以旋转矩阵后加上当前位置得到夹爪的世界坐标
            grasp_position = cur_position + self.d
            self.obj_prim.set_world_pose(position=np.array(grasp_position))
            world.step(render=True)    

    
    def release_by_target(self,target,world):
        '''
        把机械臂移动到目标位置进行释放
        '''
        tmp_height = target[1]
        while abs(tmp_height - self.lift) > 0.01:
            self.dc.set_dof_position_target(self.dof_lift,self.lift + 0.01*(tmp_height - self.lift)/abs(tmp_height - self.lift))
            self.lift = self.lift + 0.01*(tmp_height - self.lift)/abs(tmp_height - self.lift)

            # 让物体跟随机械臂的升高而升高
            self.d = (-0.02, -0.4 - self.arm, 0.06 + self.lift)
            cur_position,roll,pitch,yaw = self.trans_pos()
            
            # d乘以旋转矩阵后加上当前位置得到夹爪的世界坐标
            R = self.rpy2R([roll,pitch,yaw])
            R = np.linalg.inv(R)
            self.d = np.dot(self.d,R)
            grasp_position = cur_position + self.d
            self.obj_prim.set_world_pose(position=np.array(grasp_position))
            world.step(render=True)    

        # 设置机械臂伸出去的速度为0.01，每次伸出去0.01来是物体跟随机械臂的运动
        tmp_length = 0
        while tmp_length < target[0] / 4:
            self.dc.set_dof_position_target(self.dof_arm1,tmp_length)
            self.dc.set_dof_position_target(self.dof_arm2,tmp_length)
            self.dc.set_dof_position_target(self.dof_arm3,tmp_length)
            self.dc.set_dof_position_target(self.dof_arm4,tmp_length)
            tmp_length += 0.01
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
            self.dc.set_dof_position_target(self.dof_grasp_left,i/10) # 张开夹爪
            self.dc.set_dof_position_target(self.dof_grasp_right,i/10) 
            world.step(render=True)

        self.is_grasping = False  # 设置抓取状态为false 

        self.dc.set_dof_position_target(self.dof_arm4,0)
        self.dc.set_dof_position_target(self.dof_arm3,0)
        self.dc.set_dof_position_target(self.dof_arm2,0)
        self.dc.set_dof_position_target(self.dof_arm1,0)
        self.arm = 0 # 更新arm长度
        world.step(render=True)

        for i in range(100,0,-1):
            self.dc.set_dof_position_target(self.dof_grasp_left,i * 0.1) # 关闭夹爪
            self.dc.set_dof_position_target(self.dof_grasp_right,i * 0.1) 
            world.step(render=True)


    def trans_pos(self):
        stretch_baselink_pos = Robot(self.robot)
        position, quaternion  = stretch_baselink_pos.get_world_pose()
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        return position, roll, pitch , yaw
        

