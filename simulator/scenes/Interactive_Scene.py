from typing import Any, Dict, List, Optional
from simulator.core.scene import BaseScene
from simulator.core.config import SceneConfig
from simulator.core.register import registry

@registry.register_scene
class Interactive_Scene(BaseScene):
    # 相比于BaseScene,增加对应场景的可行驶区域，以及物体的状态获取，同时将给出一个函数能够随机生成机器人的放置位置
    def __init__(self, config:SceneConfig, scene_id:int=0):
        super().__init__(config, scene_id)
        

    def _get_uuid(self, *args: Any, **kwargs: Any) -> str:
        return "Interactive_Scene"


        