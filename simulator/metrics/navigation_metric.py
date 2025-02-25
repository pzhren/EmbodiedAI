from simulator.core.task import BaseMetric
from simulator.core.register import registry
from simulator.core.config import MetricConfig

@registry.register_metric
class NE(BaseMetric):
    def __init__(self, config: MetricConfig):
        super().__init__()
        self.config = config
   
    def calculate_metrics(self, pos1: Sequence[float], pos2: Sequence[float]) -> float:
     """Calculate navigation error (Euclidean distance between two points).
    
    Args:
        pos1: First position coordinates (x, y, z)
        pos2: Second position coordinates (x, y, z)
        
    Returns:
        Euclidean distance between the positions
    """
    return np.linalg.norm(np.array(pos1) - np.array(pos2))
    
@registry.register_metric
class SPL(BaseMetric):
    def __init__(self, config: MetricConfig):
        super().__init__()
        self.config = config
    
    def calculate_metrics(self, path_length: float, optimal_length: float, is_success: bool) -> float:
        """Compute Success weighted by Path Length (SPL) metric.
        Args:
        path_length: Actual path length traveled
        optimal_length: Shortest possible path length
        is_success: Whether the navigation was successful

        Returns:
        SPL value (0 if failed, optimal/path ratio if successful)
        """
    return optimal_length / path_length if is_success else 0.0


