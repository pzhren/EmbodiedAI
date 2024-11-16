from omni.isaac.dynamic_control import _dynamic_control
from transformations import euler_from_quaternion,quaternion_from_euler
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.robots import Robot
import math
import time
import numpy as np

class BaseControl:
    def __init__(self,robot,obj_prim=None,next_obj_prim=None):
        """
        初始化
        :param robot: 机器人usd路径
        """
        self.robot = robot
        parts = robot.rsplit('/', 1)
        # self.prim = XFormPrim(parts[0], name=parts[1])
        self.prim = XFormPrim(self.robot)

        # 默认设定线速度为0.1米/秒，角速度为0.01弧度/秒
        self.linear_velocity = 0.02
        self.angular_velocity = 0.02
        
        # 容许误差
        self.error_dis = 0.02
        self.error_ang = 0.02

        pos_static, roll_static, pitch_static, yaw_static = self.trans_pos()
        x_static,y_static,z_static = pos_static[0],pos_static[1],pos_static[2]
        self.z = z_static + 0.02 # 设置高度为原本高度+0.02,为了让机器人不和地面发生碰撞产生意外
        self.obj = obj_prim # 要抓取的物体
        self.next_obj = next_obj_prim # 要释放的桌子
        self.grasp_pos = "/World/stretch/link_gripper_s3_body/joint_grasp_center"
    
    
    def trans_pos(self):
        '''
        获取机器位置
        '''
        stretch_baselink_pos = Robot(self.robot)
        position, quaternion  = stretch_baselink_pos.get_world_pose()
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        return position, roll, pitch, yaw


    def trans2pi(self,num):
        '''
        将角度差转换到-pi到pi的区间内
        '''
        while num <-math.pi or num > math.pi:
            if num < -math.pi:
                num += 2*math.pi
            else:
                num -= 2*math.pi
        return num


    def move_by_target(self, target, world,is_grasp=False,orientation=True):
        """
        移动到指定位姿
        :param target: 目标坐标以及角度（米，米，弧度）
        :param target: isaac sim world
        :param is_grasp: 判断机器是否抓着物品
        :param orientation: 判断是否需要末端调整位置和姿态
        """
        # 获取当前的位置，和偏航角等
        pos, roll, pitch, yaw = self.trans_pos()
        x1, y1 , z1 = pos
        x2, y2, w2 = target
        angle_to_target = math.atan2(y2 - y1, x2 - x1)
        angle_diff = self.trans2pi(angle_to_target - yaw) # 转换角度到-pi~pi

        pos_static, roll_, pitch_, yaw_ = self.trans_pos()
        x_static,y_static,z_static = pos_static
        # obj_prim = XFormPrim(self.obj)
        # obj_prim = self.obj
        # 转向
        while abs(angle_diff) > self.error_ang:
            euler2q = quaternion_from_euler(roll, pitch, yaw + self.angular_velocity * angle_diff / abs(angle_diff) ) # 固定转动单位数值
            self.prim.set_world_pose(orientation=np.array(euler2q),position=np.array([x_static, y_static, self.z]))
            # 检查物品是否已经被夹取
            if is_grasp:
                grasp_pos = Robot(self.grasp_pos) # 得到夹爪的位置
                grasp_position, quaternion  = grasp_pos.get_world_pose()
                
                # 让物品跟随夹爪的位置不断转动
                pos_obj,ori_obj = self.obj.get_world_pose()
                roll_obj, pitch_obj, yaw_obj = euler_from_quaternion(ori_obj)
                euler2q_obj = quaternion_from_euler(roll_obj, pitch_obj, yaw_obj+self.angular_velocity*angle_diff/abs(angle_diff)) # 每次转0.02
                self.obj.set_world_pose(position=np.array(grasp_position),orientation=euler2q_obj)
            
            pos, roll, pitch, yaw = self.trans_pos()
            x1, y1 = pos[0], pos[1]
            angle_to_target = math.atan2(y2 - y1, x2 - x1)
            angle_diff = self.trans2pi(angle_to_target - yaw) # 转换角度到-pi~pi
            world.step(render=True)



        pos, roll, pitch, yaw = self.trans_pos()
        x1,y1,z = pos
        # 移动 
        while math.sqrt((x2 - x1)**2 + (y2 - y1)**2) > self.error_dis: 
            dx = x2 - x1
            dy = y2 - y1
            self.prim.set_world_pose(position=np.array([x1 + self.linear_velocity*dx/math.sqrt(dx**2 + dy**2),y1 + self.linear_velocity*dy/math.sqrt(dx**2 + dy**2), self.z]))
            # 检查物品是否已经被夹取
            if is_grasp:
                grasp_pos = Robot(self.grasp_pos) # 得到夹爪的位置
                grasp_position, quaternion  = grasp_pos.get_world_pose()
                # 让物品跟随夹爪的位置不断转动
                self.obj.set_world_pose(position=np.array(grasp_position))
            world.step(render=True)
        
            pos, roll, pitch, yaw = self.trans_pos()
            x1, y1 = pos[0], pos[1]


        # 最后转向
        if (orientation):# 这里加一个判断是为了在导航过程中不用频繁地调整姿态
            final_angle_diff = w2 - yaw
            final_angle_diff = self.trans2pi(final_angle_diff) # 转换角度到-pi~pi
    
            while abs(final_angle_diff) >self.error_ang:
                euler2q = quaternion_from_euler(roll, pitch, yaw+self.angular_velocity*final_angle_diff/abs(final_angle_diff)) # 每次转0.02
                self.prim.set_world_pose(orientation=np.array(euler2q))
                # 检查物品是否已经被夹取
                if is_grasp:
                    grasp_pos = Robot(self.grasp_pos) # 得到夹爪的位置
                    grasp_position, quaternion  = grasp_pos.get_world_pose()
                    # 让物品跟随夹爪的位置不断转动
                    pos_obj,ori_obj = self.obj.get_world_pose()
                    roll_obj, pitch_obj, yaw_obj = euler_from_quaternion(ori_obj)
                    euler2q_obj = quaternion_from_euler(roll_obj, pitch_obj, yaw_obj+self.angular_velocity*angle_diff/abs(angle_diff)) # 每次转0.02
                    self.obj.set_world_pose(position=np.array(grasp_position),orientation=euler2q_obj)
                
                pos, roll, pitch, yaw = self.trans_pos()
                final_angle_diff = w2 - yaw
                final_angle_diff = self.trans2pi(final_angle_diff) # 转换角度到-pi~pi
                world.step(render=True)
        print("current_pos:",pos)

    def turn_by_mode(self, target, world,is_grasp = False):
        """
        移动到指定位姿
        :param target: 转向角度数值，向左为正，向右为负
        :param world: 世界句柄
        """
        # obj_prim = XFormPrim(self.obj)                # 定义物体的prim
        # obj_prim = self.obj
        # 获取当前的位置，和偏航角等
        pos, roll, pitch, yaw = self.trans_pos()
        newpos=[pos[0],pos[1],pos[2]+0.02]
        prim = XFormPrim("/World/stretch", name="stretch")
        # 转向
        final_angle_diff = target
        final_angle_diff = self.trans2pi(final_angle_diff) # 转换角度到-pi~pi

        while abs(final_angle_diff) >self.error_ang:
            euler2q = quaternion_from_euler(roll, pitch, yaw+0.01*final_angle_diff/abs(final_angle_diff)) # 每次转0.02
            prim.set_world_pose(orientation=np.array(euler2q),position=newpos)
            # 检查物品是否已经被夹取
            if is_grasp:
                grasp_pos = Robot(self.grasp_pos) # 得到夹爪的位置
                grasp_position, _  = grasp_pos.get_world_pose()
                # 让物品跟随夹爪的位置不断转动
                self.obj.set_world_pose(position=np.array(grasp_position))
            
            pos, roll, pitch, yaw = self.trans_pos()
            final_angle_diff = self.trans2pi(final_angle_diff - 0.01*final_angle_diff/abs(final_angle_diff)) # 转换角度到-pi~pi
            world.step(render=True)
    

    def turn4grasp(self,world):
        '''
        转动到合适的位置以便于机械臂抓取
        '''
        # 获取当前的位置，和偏航角等
        # obj_prim = XFormPrim(self.obj)  # 定义物体的prim
        # obj_prim = self.obj
        pos, roll, pitch, yaw = self.trans_pos()
        x1, y1 , z1 = pos[0], pos[1], pos[2]
        target, orientation = self.obj.get_world_pose()
        x2, y2, w2 = target
        dx = x2 - x1
        dy = y2 - y1
        angle_to_target = math.atan2(dy, dx)
        angle_diff = angle_to_target - yaw + math.pi/2
        angle_diff = self.trans2pi(angle_diff) # 转换角度到-pi~pi


        # 转向
        while abs(angle_diff) > self.error_ang:
            euler2q = quaternion_from_euler(roll, pitch, yaw+self.angular_velocity*angle_diff/abs(angle_diff)) # 每次转0.02
            self.prim.set_world_pose(orientation=np.array(euler2q))  
            pos, roll, pitch, yaw = self.trans_pos()
            x1, y1 = pos[0], pos[1]
            dx = x2 - x1
            dy = y2 - y1
            angle_to_target = math.atan2(dy, dx)
            angle_diff = angle_to_target - yaw + math.pi/2
            angle_diff = self.trans2pi(angle_diff) # 转换角度到-pi~pi
            world.step(render=True)


    def turn4release(self,world,is_grasp=True,obj_prim=None):
        '''
        转动到合适的位置以便于机械臂释放
        '''
        # 获取当前的位置，和偏航角等
        pos, roll, pitch, yaw = self.trans_pos()
        x1, y1 , z1 = pos
        target, _ = obj_prim.get_world_pose()
        x2, y2, _ = target
        angle_to_target = math.atan2(y2 - y1, x2 - x1)
        angle_diff = self.trans2pi(angle_to_target - yaw + math.pi/2) # 转换角度到-pi~pi,这里加pi/2是为了让夹爪正对物品

        # grasp_obj_prim = XFormPrim(self.obj)
        # grasp_obj_prim = self.obj
        # 转向
        while abs(angle_diff) > self.error_ang:
            euler2q = quaternion_from_euler(roll, pitch, yaw+self.angular_velocity*angle_diff/abs(angle_diff)) # 每次转0.02
            self.prim.set_world_pose(orientation=np.array(euler2q))
            world.step(render=True)
            # 检查物品是否已经被夹取
            if is_grasp:
                grasp_pos = Robot(self.grasp_pos) # 得到夹爪的位置
                grasp_position, quaternion  = grasp_pos.get_world_pose()
                # 让物品跟随夹爪的位置不断转动
                _,ori_obj = self.obj.get_world_pose()
                roll_obj, pitch_obj, yaw_obj = euler_from_quaternion(ori_obj)
                euler2q_obj = quaternion_from_euler(roll_obj, pitch_obj, yaw_obj+self.angular_velocity*angle_diff/abs(angle_diff)) # 每次转0.02
                self.obj.set_world_pose(position=np.array(grasp_position),orientation=euler2q_obj)

            pos, roll, pitch, yaw = self.trans_pos()
            x1, y1, z1 = pos
            angle_to_target = math.atan2(y2 - y1, x2 - x1)
            angle_diff = self.trans2pi(angle_to_target - yaw + math.pi/2) # 转换角度到-pi~pi
            world.step(render=True)



            
