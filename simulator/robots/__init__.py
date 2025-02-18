from .franka import Franka
from .gen3_robotiq85 import Gen3_robotiq85
from simulator.core.register import registry
from .stretch import Stretch

def make_robot(id_robot, *args):
    robot = registry.get_robot(id_robot)
    assert robot is not None, "Could not find robot with name {}".format(
        id_robot
    )
    robot_ins = robot(*args)
    return robot_ins