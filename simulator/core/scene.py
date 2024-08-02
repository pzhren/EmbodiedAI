from abc import ABC 
from typing import Tuple
from simulator.core.config import SceneConfig
import numpy as np

from lazyimport import lazyimport
lazyimport(
    globals(), 
    """
    from omni.isaac.core.scenes import Scene as OmniBaseScene
    """
)
from abc import ABC

class ObjectBase(ABC):
    def __init__():
        pass

class BaseScene(OmniBaseScene, ABC):
    """
    Provide methods to add objects of interest in the stage to retrieve their information 
    and set their reset default state in an easy way
    """
    def __init__(self, config:SceneConfig):
        
        self._scene_file = config.scene_file
        self._use_floor_plane = config.use_floor_plane
        self._floor_plane_visible = config.floor_plane_visible
        self._use_sky_box = config.use_sky_box
        self.config = config
        self._objects = []
        
    def _load_usd_scene(self, prim_path_root:str="Scene"):
        #load usd scene
        if self._scene_file.endswith(".usd") or self._scene_file.endswith(".usda") or self._scene_file.endswith(".usdc"):
            return self._scene_file, "/"+prim_path_root
    
    def _load_scene(self):
        pass
    
    def _add_object(self, obj:BaseObject):
        """
        add object to scene
        """
        return super.add_object()
    
    def compute_object_AABB(self,name:str)->Tuple[np.ndarray, np.ndarray]:
        """
        compute object AABB
        """
        return super.compute_object_AABB(name)

    def clear(self)->None:
        """
        clear all objects
        """
        super.clear()

    @property
    def floor_plane(self):
        """
        get floor plane
        """
        return self._floor_plane
    
    @property
    def objects(self)->dict:
        """
        get objects in scene
        """
        return self._objects
    
