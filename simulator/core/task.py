from abc import ABC, abstractmethod
import isaacsim
from typing import Dict, Any
from simulator.utils.log_utils import create_module_log
from simulator.core.config import TaskConfig

from lazyimport import lazyimport
lazyimport(
    globals(),
    """
    from omni.isaac.core.tasks import BaseTask as OmniBaseTask
    """
)


class BaseMetric(ABC):
    def __init__(self):
        pass

    def calculate(self, task, observations) -> float:
        raise NotImplementedError

class BaseTask(ABC):
    """
    wrap of omniverse isaac sim's base task

    * enable register for auto register task
    * contains robots
    """
    tasks = {}

    def __init__(self, config: TaskConfig):
        self.objects = None
        self.robots = None
        name = config.name
        offset = config.offset
        super().__init__(name=name, offset=offset)
        self.config = config

        self.metrics: dict[str, BaseMetric] = {}
        self.steps = 0
        self.work = True

        for metric_config in config.metrics:
            self.metrics[metric_config.name] = create_metric(metric_config)

    def step(self):
        pass 

    def get_observations(self) -> Dict[str, Any]:
        """
        Returns current observations from the objects needed for the behavioral layer.

        Return:
            Dict[str, Any]: observation of robots in this task
        """
        if not self.work:
            return {}
        obs = {}
        for robot_name, robot in self.robots.items():
            try:
                obs[robot_name] = robot.get_obs()
            except Exception as e:
                log.ERROR(self.name)
                log.ERROR(e)
                traceback.print_exc()
                return {}
        return obs

    def update_metrics(self):
        for _, metric in self.metrics.items():
            metric.update()

    def calculate_metrics(self) -> dict:
        metrics_res = {}
        for name, metric in self.metrics.items():
            metrics_res[name] = metric.calculate()

        return metrics_res

    def _reset_variables(self, env):
        """
        Task-specific internal variable reset

        Args:
            env (Environment): environment instance
        """
        # By default, reset reward, done, and info
        self._reward = None
        self._done = False
        self._success = False
        self._info = None


    @abstractmethod
    def is_done(self) -> bool:
        """
        Returns True of the task is done.

        Raises:
            NotImplementedError: this must be overridden.
        """
        raise NotImplementedError
    
    @abstractmethod
    def individual_reset(self):
        """
        reload this task individually without reloading whole world.
        """
        raise NotImplementedError

    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        """called before stepping the physics simulation.

        Args:
            time_step_index (int): [description]
            simulation_time (float): [description]
        """
        self.steps += 1
        return

    def post_reset(self) -> None:
        """Calls while doing a .reset() on the world."""
        self.steps = 0
        for robot in self.robots.values():
            robot.post_reset()
        return



