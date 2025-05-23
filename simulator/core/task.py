from abc import ABC, abstractmethod
import isaacsim
from typing import Dict, Any
from simulator.utils.log_utils import create_module_log
from simulator.core.config import TaskConfig
from simulator.metrics import make_metric
from lazyimport import lazyimport
lazyimport(
    globals(),
    """
    from omni.isaac.core.tasks import BaseTask as OmniBaseTask
    """
)




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
        self.name = config.name
        self.offset = config.offset
        self.config = config
        self.metrics: dict[str, BaseMetric] = {}
        self.steps = 0
        self.work = True
        self._success = False
        for metric_config in config.metrics:
            self.metrics[metric_config.name] = make_metric(metric_config.type, metric_config)

    def step(self):

        pass 

    def get_observations(self) -> Dict[str, Any]:
        """
        Returns current observations from the objects needed for the behavioral layer.

        Return:
            Dict[str, Any]: observation of robots in this task
        """
        obs = {}
        return obs

    def update_metrics(self):
        self._info = {}
        for name, metric in self.metrics.items():
            self._info[name] = metric.update(self)
    
    def init(self, robots, objects):
        self.robots = robots
        self.objects = objects
        self._reset_variables()

    def _reset_variables(self):
        """
        Task-specific internal variable reset
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



