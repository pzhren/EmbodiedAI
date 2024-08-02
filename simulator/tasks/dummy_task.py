from simulator.core.tasks import BaseTask
from simulator.core.configs import TaskConfig

class DummyTask(BaseTask):
    def __init__(self,config:TaskConfig,Scene):
        super().__init__(config,None)

    