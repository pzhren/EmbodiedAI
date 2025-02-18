from typing import Optional, Callable, Type, DefaultDict, Any, Dict
import collections

MODULE={
    "tasks",
    "objects",
    "sensors",
    "devices",
    "robots",
    "scenes"
    "controllers",
    "metrics"
}
class Singleton(type):
    _instances: Dict["Singleton", "Singleton"] = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(
                *args, **kwargs
            )
        return cls._instances[cls]

class Registry(metaclass=Singleton):
    """
    Register class for registering objects, tasks, sensors, devices, metrics, and controllers.
    """
    mapping: DefaultDict[str, Any] = collections.defaultdict(dict)
    
    @classmethod
    def _check_register(cls, target, _type:str, name:str, assert_type:Optional[Type]=None):
        if not callable(target):
            raise Exception(f"Error:{target} must be callable!")
        register_name = target.__name__.lower()  if name is None else name
        if register_name in cls.mapping[_type] :
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
            register_name = target.__name__ if name is None else name
            cls.mapping[_type][register_name] = target
            return target
        
        if target is None:
            return _register()
        else:
            cls._check_register(target=target, _type=_type, name=name, assert_type=assert_type)
            return _register(target)

    @classmethod
    def register_task(cls, to_register, *, name:Optional[str] = None):
        from simulator.core.task import BaseTask
        return cls._register_impl("tasks", to_register, name, assert_type=BaseTask)

    @classmethod
    def register_object(cls, to_register, *, name:Optional[str] = None):
        from simulator.core.prim import BaseObject
        return cls._register_impl("objects", to_register, name, assert_type=BaseObject)
    
    @classmethod
    def register_sensor(cls, to_register, *, name:Optional[str] = None):
        from simulator.core.sensor import BaseSensor
        return cls._register_impl("sensors", to_register, name, assert_type=BaseSensor)
    
    @classmethod
    def register_device(cls, to_register, *, name:Optional[str] = None):
        from simulator.core.device import BaseDevice
        return cls._register_impl("devices", to_register, name, assert_type=BaseDevice)
    
    @classmethod
    def register_metric(cls,to_register, *, name:Optional[str] = None):
        from simulator.core.task import BaseMetric
        return cls._register_impl("metrics", to_register, name, assert_type=BaseMetric)
    
    @classmethod
    def register_robot(cls, to_register, *, name:Optional[str] = None):
        from simulator.core.robot import BaseRobot
        return cls._register_impl("robots", to_register, name, assert_type=BaseRobot)
    
    @classmethod
    def register_scene(cls, to_register, *, name:Optional[str] = None):
        from simulator.core.scene import BaseScene
        return cls._register_impl("scenes", to_register, name, assert_type=BaseScene)
    
    @classmethod
    def register_controller(cls, to_register, *, name:Optional[str] = None):
        from simulator.core.robot import BaseController
        return cls._register_impl("controllers", to_register, name, assert_type=BaseController)

    @classmethod
    def modules(cls):
        return cls.mapping
    
    @classmethod
    def _get_impl(cls, _type: str, name: str) -> Type:
        return cls.mapping[_type].get(name, None)

    @classmethod
    def get_device(cls, name: str) -> Type:
        return cls._get_impl("devices", name)

    @classmethod
    def get_task(cls, name: str) -> Type:
        return cls._get_impl("tasks", name)
    
    @classmethod
    def get_robot(cls, name: str) -> Type:
        return cls._get_impl("robots", name)
    
    @classmethod
    def get_object(cls, name: str) -> Type:
        return cls._get_impl("objects", name)
    
    @classmethod
    def get_sensor(cls, name: str) -> Type:
        return cls._get_impl("sensors", name)

    @classmethod
    def get_metric(cls, name: str) -> Type:
        return cls._get_impl("metrics", name)
    
    @classmethod
    def get_configs(cls, name:str) -> Type:
        return cls._get_impl("configs", name)
    
    @classmethod
    def get_scene(cls, name:str) ->Type:
        return cls._get_impl("scenes", name)

    @classmethod
    def get_controller(cls, name:str) -> Type:
        return cls._get_impl("controllers", name)
    
registry = Registry()
