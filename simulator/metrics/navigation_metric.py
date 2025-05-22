from simulator.core.metric import BaseMetric
from simulator.core.register import registry
from simulator.core.config import MetricConfig
from collections.abc import Sequence
import numpy as np

@registry.register_metric
class NE(BaseMetric):
    def __init__(self, config: MetricConfig):
        super().__init__()
        self.config = config
        self.goal_pos = None
        self.ne = None
    
    @property
    def name(self):
        return "NE"

    def update(self, task):

        if self.goal_pos is None:
            self.goal_pos = [task.objects[0].get_world_pose(), task.objects[1].get_world_pose()]
        if self.ne is None:
            self.ne = [[0 for _ in range(len(task.objects))] for _ in range(len(task.robots))]
        for i in range(len(task.robots)):
            pos = task.robots[i].get_world_pose()
            if task.stage[i] ==2:
                return self.ne
            self.ne[i][task.stage[i]] = self.calculate_metrics(pos,self.goal_pos[task.stage[i]][0])
        return self.ne
    

    def calculate_metrics(self, pos1: Sequence[float], pos2: Sequence[float]) -> float:
        """Calculate navigation error (Euclidean distance between two points).
        
        Args:
            pos1: First position coordinates (x, y, z)
            pos2: Second position coordinates (x, y, z)
            
        Returns:
            Euclidean distance between the positions
        """
        return np.linalg.norm(np.array(pos1) - np.array(pos2))


@registry.register_metric
class SPL(BaseMetric):
    def __init__(self, config: MetricConfig):
        super().__init__()
        self.config = config
        self.first_success = False
        self.second_success = False
        self.spl = None

    
    @property  
    def name(self):
        return "SPL"
    
    def update(self, task):
        if self.spl is None:
             self.spl = [[0, 0] for _ in range(len(task.robots))]
        for i in range(len(task.robots)):
            if task.stage[i] == 1 and task._success[i][0] and self.first_success!=True:
                self.spl[i][task.stage[i]-1] = self.calculate_metrics(task.robots[i].get_path_length(), task.optimal_length[i][task.stage[i]-1], task._success[i][0])
                self.first_success = True
            if task.stage[i] == 2 and task._success[i][1] and self.second_success!=True:
                self.spl[i][task.stage[i]-1] = self.calculate_metrics(task.robots[i].get_path_length(), task.optimal_length[i][task.stage[i]-1], task._success[i][1])
                self.second_success = True
        return self.spl
    
    def calculate_metrics(self, path_length: float, optimal_length: float, is_success: bool) -> float:
        """Compute Success weighted by Path Length (SPL) metric.
        Args:
        path_length: Actual path length traveled
        optimal_length: Shortest possible path length
        is_success: Whether the navigation was successful

        Returns:
        SPL value (0 if failed, optimal/path ratio if successful)
        """
        return optimal_length / path_length if is_success else 0.0


@registry.register_metric
class PL(BaseMetric):

    def __init__(self, config: MetricConfig):
        super().__init__()
        self.config = config
        self.init_pos = None
       

    @property
    def name(self):
        return "PL"

    def update(self, task):
        self.abs_path_length = [0 for _ in range(len(task.robots))]
        self.geo_path_length = [0 for _ in range(len(task.robots))]
        for i in range(len(task.robots)):
            if self.init_pos is None:
                self.init_pos = []
                self.init_pos.append(task.robots[0].get_world_pose())
            else:
                self.abs_path_length[i] = self.calculate_metrics(self.init_pos[i], task.robots[i].get_world_pose())
                self.geo_path_length[i] = task.robots[i].get_path_length()
        return self.abs_path_length, self.geo_path_length

    def calculate_metrics(self, start_point, final_point):
        return np.linalg.norm(np.array(start_point) - np.array(final_point))


@registry.register_metric
class CR(BaseMetric):
    def __init__(self, config: MetricConfig):
        super().__init__()
        self.config = config
        self.collision_threshold = config.collision_threshold
        self.collision_history = []
    
    @property
    def name(self):
        return "CR"

    def update(self, task):
        self.pos1 = task.robots[0].get_world_pose()
        self.pos2 = task.robots[1].get_world_pose()
        if np.linalg.norm(np.array(self.pos1) - np.array(self.pos2)) < self.collision_threshold:
            self.collision_history.append(True)
        else:
            self.collision_history.append(False)
        return self.calculate_metrics

    def calculate_metrics(self):
        return any(self.collision_history)