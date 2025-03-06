from simulator.core.task import BaseTask
from simulator.core.config import TaskConfig
from simulator.core.register import registry
from simulator.extenstions.navigate_python.map_show import *
from simulator.extenstions.navigate_python.navigate import *
from simulator.extenstions.navigate_python.dstar_lite import *
from simulator.extenstions.navigate_python.discretize_map import *
from lazyimport import lazyimport
lazyimport(globals(), """
    from omni.isaac.core.prims import XFormPrim
    from omni.isaac.core.robots import Robot
    from transformations import euler_from_quaternion,quaternion_from_euler
  """
)
@registry.register_task
class NavigateTask(BaseTask):
    def __init__(self, config:TaskConfig):
        super().__init__(config)
        # 获取任务的config, 任务的config是一个字典，包含了任务的所有信息, 例如任务的名字，任务的目标物体的名字，任务的目标物体的ID等
        self.task_config = config
        
        # 导航任务独有属性
        self.start_points = config.start_points
        self.goal_points = config.goal_points
        self.reached_goal = False
        self.max_steps = config.max_steps
        self.goal_threshold = config.goal_threshold
        self.object_ids = config.object_ids
        self.map_path = config.map_path[0]
        self.get_map(self.map_path)
    
    
    def get_task_type(self):
        '''
        Get the type of the task, e.g. "navigate", "manipulate”
        '''
        return self.task_config["task_type"]
    
    
    def get_obj_ID(self):
        '''
        Get the obj ID in the task, if more than one object, return a list of obj ID
        '''
        return self.task_config["obj_ID"]
    
    
    def get_obj_name(self):
        '''
        Get the name of the obj in the task, if more than one object, return a list of obj name
        '''
        return self.task_config["obj_name"]
    
    
    def get_task_contnet(self):
        '''
        Get the content of the task, e.g. "pick up", "put down", "navigate to"
        '''
        return self.task_config["task_content"]
    
    
    def get_robot_name(self):
        '''
        Get the name of the robot in the task
        '''
        return self.task_config["robot_name"]
    
    def get_observations(self):
        obs = []
        for robot in self.robots:
            for sensor in robot.sensors:
                obs.append(sensor.data)
        return obs

    def step(self):
        """
        return obs reward done info
        """
        super().step()
        observations = self.get_observations()
        self.update_metrics()
        self.steps+=1
        # self.is_done()
        return observations, self.metrics, self._success

    def is_done(self) -> bool:
        """
        检查导航任务是否完成
        """
        for robot_id in range(len(self.robots)):
            robot_position = self.robots[robot_id].get_world_pose()
        # 检查机器人是否到达目标点
            if self._is_at_goal(robot_position):
                self.reached_goal = True
                self._done = True
                self._success = True
                return True
        else:
            self.reached_goal = False
            self._done = False
            self._success = False
        # 检查是否超过最大步数
        if self.steps >= self.max_steps:
            self._done = True
            self._success = False
            return False


    def _is_at_goal(self, position, id, goal_threshold=0.8):
        """
        检查机器人是否到达目标点
        """

        distance = ((position[0] - self.goal_point[0]) ** 2 +
                    (position[1] - self.goal_point[1]) ** 2) ** 0.5
        return distance < self.goal_threshold


    def individual_reset(self):
        """
        重置导航任务
        """
        self.robot.reset_position(self.start_point)
        self.reached_goal = False
        self.steps = 0
        self._reset_variables(None)


    def get_distance_to_goal(self):
        """
        获取机器人到目标点的距离
        """
        distances = {}
        robot_position = self.robot.get_position()
        distance = ((robot_position[0] - self.goal_point[0]) ** 2 +
                    (robot_position[1] - self.goal_point[1]) ** 2) ** 0.5
        distances[self.get_robot_name()] = distance
        return distances
    
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
    
    def trans_pos(self):
        '''
        获取机器位置
        '''
        stretch_baselink_pos = Robot(self.task_config.robots[0].prim_path)
        position, quaternion  = stretch_baselink_pos.get_world_pose()
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        return position, roll, pitch, yaw
    

    def get_map(self, map_path):
        self.map_path = map_path
        hm = HeightMap(map_path)
        xy_range = hm.compute_range()
        hm.make_map()
        hm_map = hm.get_map()

        self.navigator = Navigator(area_range=xy_range, map=hm_map, scale_ratio=1)
        self.navigator.planner.compute_cost_map()
        return self.navigator

    def get_distance(self, goal_pos):
        '''
        根据机器人位置和物品位置计算规划路径距离
        robot_pos和goal_pos都是isaacsim的世界坐标
        '''
        # 如果地图路径发生变化，则重新获取地图
        # if self.map_path != map_path:
        #     self.get_map(map_path)
        #     # hm = HeightMap(map_path)
        #     # xy_range = hm.compute_range()
        #     # hm.make_map()
        #     # hm_map = hm.get_map()

        #     # navigator = Navigator(area_range=xy_range, map=hm_map, scale_ratio=1)
        #     # navigator.planner.compute_cost_map()
        
        # show_map_(navigator.planner.cost_map)
        robot_form = XFormPrim(self.task_config.robots[0].prim_path)
        robot_pos,_ = robot_form.get_world_pose()
        robot_pos = [robot_pos[0], robot_pos[1]]
        
        path, map_nav_path = self.navigator.navigate(goal_pos, robot_pos)
        # map_goal_pos = self.navigator.planner.real2map(goal_pos)
        # show_map_(self.navigator.planner.cost_map, map_nav_path, map_goal_pos)

        # 计算路径总距离，使用 zip 将相邻点配对
        total_distance = sum(math.hypot(x2 - x1, y2 - y1)for (x1, y1), (x2, y2) in zip(path, path[1:]))
        
        # 根据路径算出下一步应该采取什么动作
        if len(path) > 1:
            next_path_point = path[1]
        else:
            next_path_point = path[0]
        
        # 通过计算向量的角度，得到下一步应该采取的方向
        angle_diff = math.atan2(next_path_point[1] - robot_pos[1], next_path_point[0] - robot_pos[0])
        angle_diff = self.trans2pi(angle_diff)
        _, _, _, yaw  = self.trans_pos()
        print("yaw", yaw)
        final_angle_diff = self.trans2pi(angle_diff - yaw)
        print("final_angle_diff", final_angle_diff)
        if -0.015 < final_angle_diff < 0.015:
            action = "w"
        elif final_angle_diff > 0:
            action = "a" 
        elif final_angle_diff < 0: 
            action = "d"
        
        return total_distance, action
    