from simulator.core.agent import BaseAgent
from gym import spaces


class RandomAgent(BaseAgent):
    def __init__(self, config):
        super().__init__(config)
        self.action_space = spaces.Tuple((spaces.Discrete(4), spaces.Box(low=0, high=1, shape=(1,))))

    def act(self, observation):
        action  = self.action_space.sample()
        return action

    def reset(self):
        pass   