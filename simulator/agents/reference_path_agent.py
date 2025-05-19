import json
import numpy as np
from simulator.core.agent import BaseAgent
from gym import spaces
import os
import sys


class ReferencePathAgent(BaseAgent):
    def __init__(self, config):
        self.task_path = config.task.task_path
        with open(self.task_path,"r") as f:
            self.data = json.load(f)
        self.path = self.data["Reference path"]
        self.step = 0
        self.action_map = {
            "move forward": 0,
            "move backward": 1,
            "turn left": 2,
            "turn right": 3,
            "stop": 4,
        }

    def act(self, observation):
        if self.step >= len(self.path):
            return np.array([0, 0])
        action = self.action_map[self.path[self.step][0]]
        
        value = self.path[self.step][1]
        self.step += 1
        return np.array([action, value])

    def reset(self):
        self.step = 0