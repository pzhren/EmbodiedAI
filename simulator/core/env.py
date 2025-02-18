from simulator.core.simulator import Simulator
from simulator.core.config import EnvConfig
from simulator.core.register import registry
from simulator.robots import make_robot

class BaseEnv:
    
    def __init__(self, configs:EnvConfig):
        self.configs = configs.config
        self.sim  = Simulator(self.configs.sim)
       
        self.task_config = self.configs.task
        self.robots = []
        self.load()
        self.sim.play()
        self.is_running = self.sim.is_running

    def load_scene(self):
        scene_class = registry.get_scene(self.configs.scene.type)
        self.scene = scene_class(self.configs.scene)
        self.sim.import_scene(self.scene)
        # register.get(self.configs)
        # load scene and object into simulator
        pass
    
    def load_robot(self):
        robots = []
        for robot in self.task_config.robots:
            robot = make_robot(robot.type, robot)
            robots.append(robot)
            self.sim.import_robot(robot)
            robot.init()
            # intialize robot's controller

            # intialize robot's sensor
            for sensor in robot.sensors:
                sensor.init()

        return robots
    
    def load_object(self):
        objects = []
        for obj in self.task_config.objects:
            obj = make_object(obj.type, obj)
            self.objects.append(obj)
            self.sim.import_object(obj)
        
        return objects

    def load_task(self):
        self.task = make_task(self.task_config.type, self.task_config)
        robots = self.load_robot()
        objects = self.load_object()
        self.task.init(robots, objects)
        
        pass

    def load(self):
        self.load_scene()
        # self.load_robot()
        self.load_task()


    def reset(self):
        self.sim.reset()

    def step(self, action):
        pass

    def close(self):
        self.sim.close()


class RL_env(BaseEnv):
    pass
        

