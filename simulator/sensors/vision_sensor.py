from simulator.core.sensor import BaseSensor
from simulator.core.register import registry
from simulator.core.config import SensorConfig
import numpy as np
from typing import Dict
from PIL import Image
from lazyimport import lazyimport
lazyimport(globals(), """
  from omni.isaac.sensor import Camera, RotatingLidarPhysX, IMUSensor
  """
)

@registry.register_sensor
class VisionSensor(BaseSensor):
    def __init__(self, config:SensorConfig):
        super().__init__(config)
        self.config = config
        self.name = config.name
        self.type = config.type
        self.resolution = config.resolution
        self.fov = config.fov
        self.frequency = config.frequency
        self.modals = config.modals
        for modal in self.modals:
            assert modal in ["rgb", "depth", None], f"{modal} is not supported"
        
    def init(self, offset):
        """
        Create the sensor object in isaacsim
        """
        if self.on_robot:
            position = self.config.position
        else:
            position =  [x + y for x, y in zip(self.config.position, offset)]
        self.camera = Camera(
            prim_path=self.config.prim_path,
            position=position,
            orientation=self.config.orientation,
            resolution=self.resolution,
            # fov=self.fov,
            frequency=self.frequency,
            # name=self.name,
            # modals=self.config.modals,
        )
        self.camera.initialize()
        self.camera.add_motion_vectors_to_frame()
        # self.camera.add_distance_to_camera_to_frame()
        self.camera.add_distance_to_image_plane_to_frame()
        pass

    def update(self) -> Dict:
        obs = {}
        camera_data = self.camera.get_current_frame()
        if camera_data is not None:
            if "rgb" in self.modals:
                rgba_image = Image.fromarray(camera_data["rgba"].astype('uint8'))
                rgb_image = rgba_image.convert('RGB')
                rgb = np.array(rgb_image)
                obs["rgb"] = rgb
            if "depth" in self.modals:
                obs["depth"] = camera_data["distance_to_image_plane"]
        self.data = obs

    