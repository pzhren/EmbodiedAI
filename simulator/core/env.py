from simulator.core.simulator import Simulator
from simulator.core.config import EnvConfig

class BaseEnv:
    def __init__(self, configs:EnvConfig):
        self.configs = configs.config
        self.sim  = Simulator(self.configs.sim)
        self.reset()
    def reset(self):
        pass
    def step(self, action):
        pass

    def close(self):
        self.sim.close()
        

