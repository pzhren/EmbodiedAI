# from .device_base import Device
from .keyboard import Keyboard
from simulator.core.register import Registry

def make_device(id_device: str, **kwargs):
    device = registry.get_metric(id_device)
    assert device is not None, "Could not find device with name {}".format(
        id_device)
    return device(**kwargs)