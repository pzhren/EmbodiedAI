from simulator import Simulator

class BaseEnv:
    def __init__(self, configs:dict):
        self.configs = configs
        self.sim  = Simulator(configs['simulator'])
        self.reset()
    def reset(self):
        pass
    def step(self, action):
        pass
