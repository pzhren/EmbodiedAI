from .base_config import BaseConfig
from typing import List, Optional, Union
from register import register
import numpy as np
from utils.log_utils import create_module_log
import yaml

log = create_module_log(name=__name__)

CONFIG_MODULE=
{
    "sim",
    "scene",
    
}

#TODO add more configs and complete some configs
@register.register_config()
class ObjectConfig(BaseConfig):
    pass

@register.register_config()
class NPCConfig(BaseConfig):
    pass

@register.register_config()
class MetricConfig(BaseConfig):
    pass

@register.register_config()
class ControllerConfig(BaseConfig):
    pass

@register.register_config()
class SensorConfig(BaseConfig):
    pass

@register.register_config()
class RobotConfig(BaseConfig):
    # meta info
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

@register.register_config()
class SceneConfig(BaseConfig):
    """
    Scene Config
    """
    scene_file: str
    use_floor_plane: Optional[bool] = True
    floor_plane_visible: Optional[bool] = True
    use_sky_box: Optional[bool] = True


@register.register_config()
class TaskConfig(BaseConfig):
    """
    Task Config
    """
    offset: Optional[np.ndarray]
    robots: Optional[List[RobotConfig]] = []
    objects: Optional[List[ObjectConfig]] = [] # Task revelant objects
    metrics: Optional[List[MetricConfig]] = []

@register.register_config()
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

@register.register_config()
class Config(BaseConfig):
    """
    Config
    """
    simulator: Optional[SimulatorConfig]
    scene: List[SceneConfig]
    task: List[TaskConfig]
    npc: List[NPCConfig]=[]

@register.register_config()
class EnvConfig(BaseConfig):
    """
    Env Config
    """
    def __init__(self, path:str):
        self.config_path = path
        self.config = Config()
        self.config_dict = None

    def load_config(self, path):
        if self.config_path is None:
            log.error('config path is None')
            raise ValueError("Config path is None")
        if not self.config_path.endswith('.yaml') or \
                    self.config_path.endswith('.yml'):
                log.error('Config file not end with .yaml or .yml')
                raise FileNotFoundError('Config file not end with .yaml or .yml')
        if not os.path.exists(self.config_path):
            raise ValueError("Config path does not exist")

        with open(self.config_path, 'r') as f:
            self.config_dict = yaml.load(f.read(), yaml.FullLoader)
    
    def verfiy_config(self):

        self.config_dict = 

    

    
        
