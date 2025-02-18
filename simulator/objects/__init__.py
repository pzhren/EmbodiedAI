from .controllable_object import ControllableObject
from .light_object import Light
from simulator.core.register import registry


def make_object(id_object: str, *args):
    object_class = registry.get_object(id_object)
    assert object_class is not None, f"Object {id_object} not found"
    return object_class(*args)
  