from simulator.core.prim import BasePrim
from simulator.core.config import PrimConfig
from simulator.core.register import registry

@registry.register_config(name="LightConfig")
class LightConfig(PrimConfig):
    texture:Optional[str] = None


@registry.register_object(name="Light")
class Light(BasePrim):
    def __init__(self, config:LightConfig):
        super().__init__(config)
    
    def init(self):
        pass
