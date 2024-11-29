from simulator.core.simulator import Simulator
from simulator.core.config import EnvConfig

class BaseEnv:
    def __init__(self, configs:EnvConfig):
        self.configs = configs.config
        self.sim  = Simulator(self.configs.sim)
        self.sim.play()
        self.load()

    def load_scene(self):
        # load scene and object into simulator
        pass

    def load(self):
        self.load_scene()
        self.load_robot()


    def reset(self):
        self.sim.reset()

    def step(self, action):
        pass

    def close(self):
        self.sim.close()


class RL_env(BaseEnv):
    pass
        

