from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Tuple, Union
from simulator.core.config import SensorConfig
import numpy as np

class BaseSensor(ABC):
    def __init__(self, config: SensorConfig):
        self.type = config.type
        self.on_robot = config.on_robot
        self.data = None
        pass
    @abstractmethod
    def update(self) -> np.ndarray:
        """
        update sensor data
        """
        pass
    
    @abstractmethod
    def init(self):
        pass