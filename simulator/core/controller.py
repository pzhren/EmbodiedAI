

class BaseController(ABC):
    def __init__(self, config: dict):
        self.config = config
        self.type = config.type
        self.input_limit = config.input_limit
        self.output_limit = config.output_limit
    @abstractmethod
    def get_action(self) -> np.ndarray:
        pass

