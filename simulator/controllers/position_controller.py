from simulator.core.config import ControllerConfig
from simulator.core.register import registry
from simulator.core.controller import BaseController

@registry.register_controller
class PositionController(BaseController):
    type: str = "position"
    input_limit: str = "default"
    output_limit: str = "default"
    name: str = "position_controller"
    def __init__(self, config:ControllerConfig):
        super().__init__(config)
    

    def get_action(self) -> np.ndarray:
        return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])