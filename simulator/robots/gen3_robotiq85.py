from simulator.core.robot import BaseRobot
from simulator.core.config import RobotConfig
from simulator.core.register import registry

@registry.register_robot
class Gen3_robotiq85(BaseRobot):
    def __init__(self, robot_config:RobotConfig):
        super.__init__(robot_config)
        self.prim_path = "/World/Robot/gen3_robotiq85" if self.prim_path==None else robot_config.prim_path
        


        pass

    def apply_action(self, ):
        pass

    def get_joint_names(self) -> list[str]:
        pass

    def init(self):
        pass