from simulator.core.tasks import BaseTask
from simulator.core.configs import TaskConfig

class NavigateTask(BaseTask):
    def __init__(self,config:TaskConfig,Scene):
        super().__init__(config,None)
        # 获取任务的config, 任务的config是一个字典，包含了任务的所有信息, 例如任务的名字，任务的目标物体的名字，任务的目标物体的ID等
        self.task_config = config
        
        # 导航任务独有属性
        self.start_point = config.start_point
        self.goal_point = config.goal_point
        self.reached_goal = False
        self.max_steps = config.max_steps
        self.goal_threshold = config.goal_threshold
    
    
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
    
    
    def step(self):
        """
        导航任务的每一步操作
        """
        super().step()
        # 获取机器人的观测
        observations = self.get_observations()
        # 这里可以添加机器人的导航控制逻辑
        self.robot.navigate(observations[self.get_robot_name], self.goal_point)
        # 更新指标
        self.update_metrics()
        # 检查任务是否完成
        if self.is_done() and self._success:
            return True

    def is_done(self) -> bool:
        """
        检查导航任务是否完成
        """
        robot_position = self.robot.get_position()
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


    def _is_at_goal(self, position, goal_threshold=0.8):
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
    
    
    
    