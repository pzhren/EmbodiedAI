from abc import ABC
import isaacsim
from lazyimport import lazyimport
lazyimport(globals(),"""
  from omni.isaac.core.tasks import BaseTask as OmniBaseTask
""")

from utils.log_utils import log
from scenes.scene_base import BaseScene
from metrics.metric_base import BaseMetric
from configs import TaskConfig
#根据官方文档继承，同时按照GRUtopia的结构,后续要改改

class BaseTask(ABC):
    """
    wrap of omniverse isaac sim's base task

    * enable register for auto register task
    * contains robots
    """
    tasks = {}

    def __init__(self, config: TaskConfig, scene: BaseScene):
        self.objects = None
        self.robots = None
        name = config.name
        offset = config.offset
        super().__init__(name=name, offset=offset)
        self._scene = scene
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

    @abstractmethod
    def is_done(self) -> bool:
        """
        Returns True of the task is done.

        Raises:
            NotImplementedError: this must be overridden.
        """
        raise NotImplementedError

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



