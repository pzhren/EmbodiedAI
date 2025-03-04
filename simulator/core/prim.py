from abc import ABC 
from simulator.core.config import SceneConfig, PrimConfig
from lazyimport import lazyimport
lazyimport(globals(), """
    from omni.isaac.core.utils.prims import create_prim
    import omni.isaac.core.utils.bounds as bounds_utils
    from omni.isaac.core.prims import RigidPrim,RigidPrimView,GeometryPrim,XFormPrim
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
        self.collision = config.collision
    
    def get_local_pose(self):
        assert self.prim is not None
        self.prim.get_local_pose()
    
    def get_aabb(self):
        cache = bounds_utils.create_bbox_cache()
        self.aabb = bounds_utils.compute_aabb(cache, prim_path=self.prim_path)
        return self.aabb

    def init(self):
        self.prim = create_prim(
            prim_path=self.prim_path,
            prim_type= self.prim_type,
            usd_path=self.usd_path,
            translation=self.translation,
            orientation=self.orientation,
            scale=self.scale,
            attributes=self.attributes,
        )
        if self.collision:
            self.prim_collision = GeometryPrim(self.prim_path, collision=True)
        
        pass