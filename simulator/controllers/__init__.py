from simulator.core.register import registry

def make_controller(id_controller, **kwargs):
    controller = registry.get_controller(id_controller)
    assert controller is not None ,"Could not find controller with name {}".format(id_controller)
    return controller(*args, **kwargs)