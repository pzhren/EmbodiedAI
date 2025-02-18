from .vision_sensor import VisionSensor
from simulator.core.register import registry

def make_sensor(id_sensor, *args):
    sensor = registry.get_sensor(id_sensor)
    assert sensor is not None, "Could not find robot with name {}".format(id_sensor)
    sensor_ins = sensor(*args)
    return sensor_ins