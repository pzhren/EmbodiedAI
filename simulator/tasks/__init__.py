from .dummy_task import DummyTask
from simulator.core.register import registry

def make_task(id_task,**kwargs):
    task = registry.get_task(id_task)
    assert task is not None, "Could not find task with name {}".format(id_task)
    task_ins = task(**kwargs)
    return task_ins