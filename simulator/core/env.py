from simulator.core.sim import Simulator
from simulator.core.config import EnvConfig, Config
from simulator.core.register import registry
from simulator.robots import make_robot
from simulator.tasks import make_task
from typing import List, Dict, Union
from collections import deque
from simulator.utils.scene_utils import extract_target_ids
import json

class BaseEnv:
    def __init__(self, configs:Union[Config, EnvConfig]):
        if isinstance(configs, Config):
            self.configs = configs
        elif isinstance(configs, EnvConfig):
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
        self.task = []
        self.task_objects = []
        self.all_obj_dict = []
        self.load()
        self.hssd_item = self.sim.get_item_lookup()
        
    def load_scene(self, scene_id):
        scene_class = registry.get_scene(self.configs.scene.type)
        self.scenes.append(scene_class(self.configs.scene))
        self.sim.import_scene(self.scenes[scene_id], self.scene_id2offset[scene_id], scene_id)
        # register.get(self.configs)
        # load scene and object into simulator
        self.all_obj_dict.append(self.sim.get_all_objects(self.scenes[scene_id]))
        pass
    
    def load_robot(self, offset):
        robots = []
        for robot in self.task_config.robots:
            robot = make_robot(robot.type, robot)
            robots.append(robot)
            self.sim.import_robot(robot, offset)
            
            # intialize robot's controller

            # intialize robot's sensor
            for sensor in robot.sensors:
                sensor.init(offset)

        return robots
    
    
    def load_object(self, scene_id, offset=None):
        # 根据物品ID找到prim路径，并实例化为Xformprim,以列表形式返回
        # 首先通过读取task.json获取物品ID，然后通过find_object_by_id获取物品的prim路径
        target_ids = extract_target_ids(self.task_config.task_path)
        objects = self.sim.find_object_by_id(self.scenes[scene_id], target_ids)
        # for obj in self.task_config.object_ids:
        #     obj = make_object(obj.type, obj)
        #     objects.append(obj)
        #     self.sim.import_object(obj, offset)
        
        return objects
    
    def load_task(self, scene_id):
        self.task.append(make_task(self.task_config.type, self.task_config))
        offset = self.scene_id2offset[scene_id]
        self.robots.append(self.load_robot(offset))
        self.task_objects.append(self.load_object(scene_id, offset))
        self.task[scene_id].init(self.robots[scene_id], self.task_objects[scene_id])
        
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
        #    robot.apply_action(action_dict[robot.name])
           self.sim.physics_step(robot, action_dict[robot.name])


    def _post_step(self, scene_id):
        """Apply the post-sim-step part of an environment step, i.e. update the robot sensors."""
        for robot in self.robots[scene_id]:
            for sensor in robot.sensors:
                sensor.update()
      
        task = self.task[scene_id]
        obs, info, done = task.step()
        reward = info["NE"] if info is not None else 0
        return obs, info, reward, done

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
    
    def find_object_around(self, position, scene_id=0):
        return self.sim.find_object_around(self.scenes[scene_id], position)
    
    def get_scene_graph(self, robot_id, scene_id=0):
        
        pass

    def close(self):
        self.sim.close()


class RL_env(BaseEnv):
    pass
        

