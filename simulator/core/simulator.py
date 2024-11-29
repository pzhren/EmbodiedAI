import isaacsim
import carb
from omni.isaac.kit import SimulationApp
from typing import Any, Callable, Dict, List, Optional
from simulator.core.config import  SimulatorConfig
from simulator.utils.log_utils import create_module_log
from simulator.core.task import BaseTask
from lazyimport import lazyimport
lazyimport(globals(), """
    from omni.isaac.core import World
  """
)

def launch_app(app_config:dict, webrtc=False, native=False):
    """
    Launch the isaac sim app.
    """
    simulation_app = SimulationApp(app_config)
        
    if webrtc:
        from omni.isaac.core.utils.extensions import enable_extension  

        simulation_app.set_setting('/app/window/drawMouse', True)
        simulation_app.set_setting('/app/livestream/proto', 'ws')
        simulation_app.set_setting('/app/livestream/websocket/framerate_limit', 60)
        simulation_app.set_setting('/ngx/enabled', False)
        enable_extension('omni.services.streamclient.webrtc')

    elif native:
        from omni.isaac.core.utils.extensions import enable_extension  

        simulation_app.set_setting("/app/window/drawMouse", True)
        simulation_app.set_setting("/app/livestream/proto", "ws")
        simulation_app.set_setting("/app/livestream/websocket/framerate_limit", 120)
        simulation_app.set_setting("/ngx/enabled", False)
        enable_extension("omni.kit.livestream.native")

    return simulation_app

class Simulator():
    def __init__(self, config:SimulatorConfig, webrtc=False, native=False):
        
        self.phy_dt = 1./config.physics_frequency 
        self.render_dt = 1./config.render_frequency
        self.headless = config.headless
        self.height = config.height
        self.width = config.width
        self.hide_ui = config.hide_ui
        self.device = config.device
        self.app_config = {
            'headless': self.headless, 
            'height': self.height, 
            'width': self.width, 
            'physics_gpu': self.device,
            'active_gpu': self.device,
            }
        try:
            self._simulation_app = launch_app(self.app_config, webrtc=webrtc, native=native)
        except Exception as e:
            log.error("Can not lanuch isaac sim app.")
            raise e

        self._world = World(physics_dt=self.phy_dt, rendering_dt=self.render_dt, stage_units_in_meters=1.0)
        self._scene = self._world.scene
        self._stage = self._world.stage
        self._warm_up()

    def _warm_up(self, steps=10, render=True):
        """
        Warm up the simulator.
        """
        for _ in range(steps):
            self._world.step(render=render)
    
    def add_tasks(self, tasks:list[BaseTask]):
        for config in configs:
            task = create_task(config, self._scene)
            self._world.add_task(task)

        self._world.reset()
        self._warm_up()
    
    def import_scene(self, scene):
        assert self.is_running==False
        assert isinstance(scene,BaseScene)

        scene.load_scene(self)
    def play(self):
        return self._simulation_app.play()
    
    def step(self, render:bool=True) -> dict[str, Any]:
        return self._world.step(render=render)

    def reset(self):
        self._world.reset()

    def close(self):
        self._simulation_app.close()

    @property
    def current_tasks(self) -> dict[str, BaseTask]:
        return self._world._current_tasks

    @property
    def is_running(self) -> bool:
        return self._simulation_app.is_running

   
    
  