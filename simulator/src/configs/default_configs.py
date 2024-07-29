from .base_config import BaseConfig
from typing import List, Optional, Union
from register import register
import numpy as np

#TODO add more configs and complete some configs
class ObjectConfig(BaseConfig):
    pass

class NPCConfig(BaseConfig):
    pass

class MetricConfig(BaseConfig):
    pass

class ControllerConfig(BaseConfig):
    pass

class SensorConfig(BaseConfig):
    pass

class RobotConfig(BaseConfig):
    # meta info
    name: str
    type: str
    prim_path: str
    create_robot: bool = True

    # common config
    position: Optional[List[float]] = [.0, .0, .0]
    orientation: Optional[List[float]]
    scale: Optional[List[float]]

    # Parameters
    controller: Optional[List[ControllerConfig]] = None
    sensor: Optional[List[SensorConfig]] = None

class SceneConfig(BaseConfig):
    """
    Scene Config
    """
    scene_file: str
    use_floor_plane: Optional[bool] = True
    floor_plane_visible: Optional[bool] = True
    use_sky_box: Optional[bool] = True


class TaskConfig(BaseConfig):
    """
    Task Config
    """
    name: str
    offset: Optional[np.ndarray]
    robots: Optional[List[RobotConfig]] = []
    objects: Optional[List[Object]] = [] # Task revelant objects
    metrics: Optional[List[MetricConfig]] = []

class SimulatorConfig(BaseConfig):
    """
    SimulationApp and World Config
    """
    active_gpu: Optional[int] = 0
    height: Optional[int] = 720
    width: Optional[int] = 1280
    physics_gpu: Optional[int] = 0
    physics_dt: Optional[float | str] = None
    rendering_dt: Optional[float | str] = None
    rendering_interval: Optional[int] = None


class Config(BaseConfig):
    """
    Config
    """
    simulator: Optional[SimulatorConfig]
    scene: List[SceneConfig]
    task: List[TaskConfig]
    npc: List[NPCConfig]=[]


class EnvConfig():
    """
    Env Config
    """
    def __init__(self, path:str):
        self.config_path = path
