from abc import ABC, abstractmethod
import numpy as np
from gym import spaces

class BaseController(ABC):
    def __init__(self, config: dict):
        self.config = config
        self.type = config.type
        self.input_limit = config.input_limit
        self.output_limit = config.output_limit
    @abstractmethod
    def get_action(self) -> np.ndarray:
        pass

    @property
    def action_space(self) -> spaces.Box:
        pass
