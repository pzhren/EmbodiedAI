import isaacsim
import carb
from omni.isaac.kit import SimulationApp
from configs import  SimulatorConfig
from utils.log_utils import log

from lazyimport import lazyimport
lazyimport(globals(), """
    from omni.isaac.core.worlds import World
    from tasks.base_task import BaseTask
  """
)


class Simulator():
    def __init__(self, config:SimulatorConfig):
        
        self.phy_dt = config.physics_dt
        self.render_dt = config.render_dt
        self.headless = config.headless
        self.height = config.height
        self.width = config.width
        self._physics_gpu = config.physics_gpu
        self._simulation_app = SimulationApp(
            {'headless': self.headless, 'height': self.height, 'width': self.width, 'physics_gpu': self._physics_gpu})
        
        if webrtc:
            from omni.isaac.core.utils.extensions import enable_extension  

            self._simulation_app.set_setting('/app/window/drawMouse', True)
            self._simulation_app.set_setting('/app/livestream/proto', 'ws')
            self._simulation_app.set_setting('/app/livestream/websocket/framerate_limit', 60)
            self._simulation_app.set_setting('/ngx/enabled', False)
            enable_extension('omni.services.streamclient.webrtc')

        elif native:
            from omni.isaac.core.utils.extensions import enable_extension  

            self._simulation_app.set_setting("/app/window/drawMouse", True)
            self._simulation_app.set_setting("/app/livestream/proto", "ws")
            self._simulation_app.set_setting("/app/livestream/websocket/framerate_limit", 120)
            self._simulation_app.set_setting("/ngx/enabled", False)
            enable_extension("omni.kit.livestream.native")

        self._world = World(physics_dt=self.phy_dt, rendering_dt=render_dt, stage_units_in_meters=1.0)
        self._scene = self._world.scene
        self._stage = self._world.stage

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
    
    def step(self, render:bool=True) -> dict[str, Any]:
        return self._world.step(render=render)


    @property
    def current_tasks(self) -> dict[str, BaseTask]:
        return self._world._current_tasks
    
  