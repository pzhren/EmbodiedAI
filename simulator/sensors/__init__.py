from .vision_sensor import VisionSensor
from simulator.core.register import registry

def make_sensor(id_sensor, **kwargs):
    sensor = registry.get_sensor(id_sensor)
    assert sensor is not None, "Could not find robot with name {}".format(id_sensor)
    sensor_ins = sensor(**kwargs)
    return sensor_ins