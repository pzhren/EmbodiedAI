from typing import Optional, Callable, Type, DefaultDict, Any
import collections
from src.tasks import BaseTask, BaseMetric
from src.objects import BaseObject
from src.sensors import BaseSensor
from src.devices import Device
from src.robots import BaseRobot
from src.scenes import BaseScene
from src.configs import Config

MODULE={
    "tasks",
    "objects",
    "sensors",
    "devices",
    "robots",
    "scenes"
}
class Register(type):
    """
    Register class for registering objects, tasks, sensors, devices, metrics, and controllers.
    """
    mapping: DefaultDict[str, Any] = collections.defaultdict(dict)
    
    @classmethod
    def _check_register(cls, target, _type:str, name:str, assert_type:Optional[Type]=None):
        if not callable(target):
            raise Exception(f"Error:{target} must be callable!")
        register_name = target.__name__.lower()  if name is None else name
        if register_name in self.mapping[_type] :
            raise Exception(f"Error: {target} already registered!")
        if assert_type is not None:
                assert issubclass(
                    target, assert_type
                ), "{} must be a subclass of {}".format(
                    target, assert_type
                )

    @classmethod
    def _register_impl(cls, _type:str, target, name:str, assert_type:Optional[Type]=None) -> Callable:
        def _register(target):
            register_name = to_register.__name__ if name is None else name
            cls.mapping[_type][register_name] = to_register
            return to_register
        
        if target is None:
            return _register
        else:
            self._check_register(target, assert_type)
            return _register(target)
    def register_task(cls, to_register, *, name:Optional[str] = None):
        return cls._register_impl("tasks", to_register, name, assert_type=BaseTask)
        
    def register_object(cls, to_register, *, name:Optional[str] = None):
        return cls._register_impl("objects", to_register, name, assert_type=BaseObject)

    def register_sensor(cls, to_register, *, name:Optional[str] = None):
        return cls._register_impl("sensors", to_register, name, assert_type=BaseSensor)
    
    def register_device(cls, to_register, *, name:Optional[str] = None):
        return cls._register_impl("devices", to_register, name, assert_type=BaseDevice)
    
    def register_metric(cls,to_register, *, name:Optional[str] = None):
        return cls._register_impl("metrics", to_register, name, assert_type=BaseMetric)
    
    def register_robot(cls, to_register, *, name:Optional[str] = None):
        return cls._register_impl("robots", to_register, name, assert_type=BaseRobot)

    def register_config(cls, to_register, *, name:Optional[str] = None):
        return cls._register_impl("configs", to_register, name, assert_type=BaseConfig)
    def modules(cls):
        return cls.mapping
    

registry = Registry()
