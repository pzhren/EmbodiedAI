from abc import ABC

class BaseMetric(ABC):
    def __init__(self):
        pass

    def calculate(self, task, observations) -> float:
        raise NotImplementedError
    
    