from simulator.core.controller import BaseController
from simulator.core.config import ControllerConfig
from simulator.core.register import registry

@registry.register_controller
class IKController(BaseController):
    def __init__(self, config: ControllerConfig):
        self.super().__init__(config)
        self.config = config
        self.type = config.type
        self.input_limit = config.input_limit
        self.output_limit = config.output_limit