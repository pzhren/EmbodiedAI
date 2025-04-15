from abc import ABC, abstractmethod


class BaseAgent(ABC):
    def __init__(self, config):
        self.config = config
        
    def reset(self):
        pass

    def act(self, observation):
        pass