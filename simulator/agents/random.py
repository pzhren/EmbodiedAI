from simulator.core.agent import BaseAgent
from gym import spaces


class RandomAgent(BaseAgent):
    def __init__(self, config):
        super().__init__(config)
        self.action_space = config.action_space

    def act(self, observation):
        action  = self.action_space.sample()
        return action

    def reset(self):
        pass   