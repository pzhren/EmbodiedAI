from .dummy_task import DummyTask
from .navigation import NavigateTask
from simulator.core.register import registry

def make_task(id_task,*args):
    task = registry.get_task(id_task)
    assert task is not None, "Could not find task with name {}".format(id_task)
    task_ins = task(*args)
    return task_ins