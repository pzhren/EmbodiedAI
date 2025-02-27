from simulator.core.simulator import Simulator
from simulator.core.config import EnvConfig
from simulator.core.register import registry
from simulator.robots import make_robot
from simulator.tasks import make_task
from typing import List, Dict
from collections import deque

class BaseEnv:
    def __init__(self, configs:EnvConfig):
        self.configs = configs.config
        self.sim  = Simulator(self.configs.sim)
        self.task_config = self.configs.task
        self.sim.play()
        self.is_running = self.sim.is_running
        self.offset = self.configs.offset
        self.scene_num = self.configs.env_num
        self.scene_id = list(range(self.scene_num))
        self.scene_id2offset = {id:list(map(lambda x: id * x, self.offset)) for id in self.scene_id}
        self.robots = []
        self.scenes = []
        self.task_objects = []
        self.load()
        
    def load_scene(self, scene_id):
        scene_class = registry.get_scene(self.configs.scene.type)
        self.scenes.append(scene_class(self.configs.scene))
        self.sim.import_scene(self.scenes[scene_id], self.scene_id2offset[scene_id], scene_id)
        # register.get(self.configs)
        # load scene and object into simulator
        pass
    
    def load_robot(self, offset):
        robots = []
        for robot in self.task_config.robots:
            robot = make_robot(robot.type, robot)
            robots.append(robot)
            self.sim.import_robot(robot, offset)
            robot.init()
            # intialize robot's controller

            # intialize robot's sensor
            for sensor in robot.sensors:
                sensor.init(offset)

        return robots
    
    def load_object(self, offset):
        objects = []
        for obj in self.task_config.objects:
            obj = make_object(obj.type, obj)
            objects.append(obj)
            self.sim.import_object(obj, offset)
        
        return objects
    
    def load_task(self, scene_id):
        self.task = make_task(self.task_config.type, self.task_config)
        offset = self.scene_id2offset[scene_id]
        self.robots.append(self.load_robot(offset))
        self.task_objects.append(self.load_object(offset))
        self.task.init(self.robots[scene_id], self.task_objects[scene_id])
        
        pass

    def load(self):
        for i in self.scene_id:
            self.load_scene(i)
            # self.load_robot()
            self.load_task(i)

    def reset(self):
        self.sim.reset()
    
    def _pre_step(self, action, scene_id):
        """Apply the pre-sim-step part of an environment step, i.e. apply the robot actions."""
        if not isinstance(action, dict):
            action_dict = dict()
            idx = 0
            for robot in self.robots[scene_id]:
                action_dim = robot.action_dim
                action_dict[robot.name] = action[idx : idx + action_dim]
                idx += action_dim
        else:
            # Our inputted action is the action dictionary
            action_dict = action

        # Iterate over all robots and apply actions
        for robot in self.robots[scene_id]:
           robot.apply_action(action_dict[robot.name])


    def _post_step(self, scene_id):
        """Apply the post-sim-step part of an environment step, i.e. update the robot sensors."""
        for robot in self.robots[scene_id]:
            for sensor in robot.sensors:
                sensor.update()
        return self.task.step()

    def step(self, action):
        # if isinstance(action, Iterable) and not isinstance(action, (dict, OrderedDict)):
            # Convert numpy arrays and lists to tensors
            # Skip dict action
            # action = th.as_tensor(action, dtype=th.float).flatten()
        obs = []
        if len(action) != self.scene_num and self.scene_num==1:
            action = [action]
        assert len(action) == self.scene_num
        for i in range(self.scene_num):
            self._pre_step(action[i], scene_id=i)

        # Step simulation
        self.sim.step(render=True)
        
        for i in range(self.scene_num):
            obs.append(self._post_step(scene_id=i))
        # Run final post-processing
        return obs


    def close(self):
        self.sim.close()


class RL_env(BaseEnv):
    pass
        

