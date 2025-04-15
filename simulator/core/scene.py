from abc import ABC 
from typing import Tuple
from simulator.core.config import SceneConfig, PrimConfig
from simulator.core.prim import BasePrim
import numpy as np
import json
from lazyimport import lazyimport
lazyimport(globals(), """
    from omni.isaac.core.scenes import Scene as OmniBaseScene
    from omni.isaac.core.utils.prims import create_prim
  """
)


class BaseScene(ABC):
    """
    Provide methods to add objects of interest in the stage to retrieve their information 
    and set their reset default state in an easy way
    """
    def __init__(self, config:SceneConfig, scene_id:int=0):
        # self.scene = OmniBaseScene()
        self._scene_file = config.scene_file
        self._use_floor_plane = config.use_floor_plane
        self._floor_plane_visible = config.floor_plane_visible
        self._add_wall = config.add_wall
        self._use_sky_box = config.use_sky_box
        self.config = config
        # self._load_usd = config.load_usd_scene
        self.scene_prim_dict = {} #{prim path: attributes}
        self.scene_id = scene_id
        self._objects = []
        self._load_scene()
        
    def _load_scene(self, prim_path_root:str="/Scene"):
        #load usd scene
        if self._scene_file.endswith(".usd") or self._scene_file.endswith(".usda") or self._scene_file.endswith(".usdc"):
            self.scene_prim_dict.update(
            {"Scene":BasePrim(PrimConfig(
                prim_path = prim_path_root+str(self.scene_id),
                usd_path = self._scene_file,
                prim_type = "Xform",
                scale = [1,1,1],
                translation = [0,0,0],
                orientation = [1,0,0,0],
                collision = True
            ))})
        
        elif self._scene_file.endswith(".json"):
            with open(self._scene_file, "r") as f:
                scene_file = json.load(f)
                # 未定义使用
            # return self._scene_file, "/"+prim_path_root  

    def _load_objects(self):
        pass
    
    def _add_object(self, obj):
        """
        add object to scene
        """
        return self.scene.add_object()
    
    def compute_object_AABB(self,name:str)->Tuple[np.ndarray, np.ndarray]:
        """
        compute object AABB
        """
        return self.scene.compute_object_AABB(name)
        pass

    def clear(self)->None:
        """
        clear all objects
        """
        self.scene.clear()
        self.scene_prim_dict = {}
    
    def init(self)->None:
        """
        initialize scene after being loaded in simulator
        """
        pass

    @property
    def floor_plane(self):
        """
        get floor plane
        """
        return self.scene._floor_plane
    
    @property
    def objects(self)->dict:
        """
        get objects in scene
        """
        return self.scene._objects
    
