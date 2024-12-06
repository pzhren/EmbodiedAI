from simulator.core.robot import BaseSensor
from simulator.core.register import registry
from simulator.core.config import SensorConfig

@registry.register_sensor("vision_sensor")
class VisionSensor(BaseSensor):
    def __init__(self, config:SensorConfig):
        super().__init__()
        self.config = config
        self.name = config.name
        self.type = config.type
        self.resolution = config.resolution
        self.fov = config.fov
        self.near = config.near
        self.far = config.far
        self.image_type = config.image_type
        self.image_format =config.image_format

    