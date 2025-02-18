from simulator.core.robot import BaseRobot
from simulator.core.config import RobotConfig
from simulator.core.register import registry

@registry.register_robot
class Franka(BaseRobot):
    def __init__(self, config: RobotConfig):
        super().__init__(config)