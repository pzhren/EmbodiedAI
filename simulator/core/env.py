from simulator.core.simulator import Simulator
from simulator.core.config import EnvConfig
from simulator.core.register import registry


class BaseEnv:
    
    def __init__(self, configs:EnvConfig):
        self.configs = configs.config
        self.sim  = Simulator(self.configs.sim)
        self.sim.play()

       
        self.load()

    def load_scene(self):
        scene_class = registry.get_scene(self.configs.scene.type)
        self.scene = scene_class(self.configs.scene)
        self.sim.import_scene(self.scene)
        # register.get(self.configs)
        # load scene and object into simulator
        pass
    
    def load_robot(self):
        roborot_class = registry.get_robot(self.configs.robot.type)
        self.robot = roborot_class(self.configs.robot)
        self.sim.import_robot(self.robot)
        pass

    def load_task(self):
        pass

    def load(self):
        self.load_scene()
        self.load_robot()
        self.load_task()


    def reset(self):
        self.sim.reset()

    def step(self, action):
        pass

    def close(self):
        self.sim.close()


class RL_env(BaseEnv):
    pass
        

