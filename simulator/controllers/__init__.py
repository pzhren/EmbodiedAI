from simulator.core.register import registry
from .dd_controller import DDController
from .ik_controller import IKController
from .position_controller import PositionController
from .grasp_controller import StretchGraspController

def make_controller(id_controller, *args):
    controller = registry.get_controller(id_controller)
    assert controller is not None ,"Could not find controller with name {}".format(id_controller)
    return controller(*args)