from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Tuple, Union
from simulator.core.config import SensorConfig
import numpy as np

class BaseSensor(ABC):
    def __init__(self):
        self.type = None
        pass
    @abstractmethod
    def get_observation(self) -> np.ndarray:
        pass
    
    @abstractmethod
    def init(self):
        pass