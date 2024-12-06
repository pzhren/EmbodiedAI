from abc import ABC 
from simulator.core.config import SceneConfig, PrimConfig
lazyimport(globals(), """
    from omni.isaac.core.utils.prims import create_prim
  """
)
class BasePrim(ABC):
    def __init__(self, config:PrimConfig):
        self.config = config
        self.prim_path = config.prim_path
        self.usd_path = config.usd_path
        self.name = config.name
        self.scale = config.scale
        self.translation = config.translation
        self.orientation = config.orientation
        self.attributes = config.attributes
        self.prim_type = config.prim_type
        self.prim = None
    
    def get_local_pose():
        assert self.prim is not None
        self.prim.get_local_pose()
    
    def init(self):
        self.prim = create_prim(
            prim_path=self.prim_path,
            prim_type= self.prim_type,
            usd_path=self.usd_path,
            name=self.name,
            translation=self.translation,
            orientation=self.orientation,
            scale=self.scale,
            attributes=self.attributes,
        )
        pass