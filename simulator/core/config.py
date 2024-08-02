import os
from typing import List, Optional, Union
from pydantic import BaseModel
from simulator.utils.log_utils import create_module_log
import yaml
from copy import deepcopy
import numpy as np

from simulator.utils.config_utils import merge_config
log = create_module_log(name=__name__)


#TODO add more configs and complete some configs


class BaseConfig(BaseModel):
    """
    Define Base Configuration for all Configuration
    """
    name: Optional[str]=None
    pass

class ObjectConfig(BaseConfig):
    pass


class NPCConfig(BaseConfig):
    pass


class MetricConfig(BaseConfig):
    pass

class ControllerConfig(BaseConfig):
    type: str
    input_limit: Optional[str|List] = "default"
    output_limit: Optional[str|List] = "default"
    pass

class SensorConfig(BaseConfig):
    type: str
    pass

class RobotConfig(BaseConfig):
    # meta info
    type: str
    prim_path: str
    create_robot: bool = True

    # common config
    position: Optional[List[float]] = [.0, .0, .0]
    orientation: Optional[List[float]] = [.0, .0, .0, 1.0]
    scale: Optional[List[float]] = [1.0, 1.0, 1.0]

    # Parameters
    controller: Optional[List[ControllerConfig]] = None
    sensor: Optional[List[SensorConfig]] = None

class SceneConfig(BaseConfig):
    """
    Scene Config
    """
    scene_file: str|None
    use_floor_plane: Optional[bool] = True
    floor_plane_visible: Optional[bool] = True
    use_sky_box: Optional[bool] = True


class TaskConfig(BaseConfig):
    """
    Task Config
    """
    type: Optional[str] ="default"
    
    robots: Optional[List[RobotConfig]] = []
    objects: Optional[List[ObjectConfig]] = [] # Task revelant objects
    metrics: Optional[List[MetricConfig]] = []
    offset: Optional[List[float]] = None

class SimulatorConfig(BaseConfig):
    """
    SimulationApp and World Config
    """
    height: Optional[int] = 720
    width: Optional[int] = 1280
    device: Optional[int] = 0
    physics_frequency: Optional[float | str] = None
    render_frequency: Optional[float | str] = None
    rendering_interval: Optional[int] = None
    headless: Optional[bool] = True
    hide_ui: Optional[bool] = True

class Config(BaseConfig):
    """
    Config
    """
    sim: Optional[SimulatorConfig]
    scene: SceneConfig
    task: TaskConfig
    npcs: List[NPCConfig]=[]

# class MultiConfig(BaseConfig):
#     """
#     Multi Env Config
#     """
#     env_num: Optional[int] = 1
#     sim: Optional[SimulatorConfig]
#     scenes: List[SceneConfig]
#     tasks: List[List[TaskConfig]]
#     npcs: List[List[NPCConfig]] = [[]]

class EnvConfig():
    """
    Env Config
    """
    def __init__(self, path:str, multi_env:bool=False):
        self._env_num = 1
        self._offset_size = None
        self.config_path = path
        self.config_dict = None
        self.load_config(path)
        self.config_dict = merge_config(
            self.default_config.dict(), 
            self.config_dict,
            inplace = True, 
            verbose = True
        )
        print(self.config_dict)
        self.config = Config(**self.config_dict)

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
            print(self.config_dict)
            
        
    @property
    def default_config(self):
        """
        Returns:
            config: Default configuration for this environment. 
        """
        #Simulator kwargs
        default_sim = {
            "height": 720,
            "width": 1280,
            "device": 0, 
            "physics_frequency": 60,
            "render_frequency": 60,
            "headless": True,
        }

        #Scene kwargs
        default_scene = {
            "scene_file": None
        }
        default_task = {}
        default_npc = {}
        return Config(
            name="default",
            sim=SimulatorConfig(**default_sim),
            scene=SceneConfig(**default_scene),
            task=TaskConfig(**default_task),
            npcs=[NPCConfig(**default_npc)]
        )
    


    
        
