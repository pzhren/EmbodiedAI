from simulator.core.task import BaseTask
from simulator.core.config import TaskConfig
from simulator.core.register import registry

@registry.register_task
class DummyTask(BaseTask):
    def __init__(self, config:TaskConfig):
        super().__init__(config)
    
    def init(self,robots, objects):
        self.robots = robots    
        self.objects = objects


    def is_done(self):
        return False

    def individual_reset(self):
        pass

    