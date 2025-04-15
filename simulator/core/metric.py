from abc import ABC, abstractmethod
class BaseMetric(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def update(self, task) -> float:
        raise NotImplementedError
    
    @abstractmethod
    def calculate_metrics(self) -> float:
        raise NotImplementedError