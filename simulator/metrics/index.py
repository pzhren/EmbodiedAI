import numpy as np
from typing import Sequence, List

class Metrics:
    """Class for calculating navigation performance metrics."""
    
    def __init__(self, task: object):
        self.task = task  # Preserved for potential future use
    
        
    def NE(self, pos1: Sequence[float], pos2: Sequence[float]) -> float:
        """Calculate navigation error (Euclidean distance between two points).
        
        Args:
            pos1: First position coordinates (x, y, z)
            pos2: Second position coordinates (x, y, z)
            
        Returns:
            Euclidean distance between the positions
        """
        return np.linalg.norm(np.array(pos1) - np.array(pos2))
    
    
    def SPL(self, path_length: float, optimal_length: float, is_success: bool) -> float:
        """Compute Success weighted by Path Length (SPL) metric.
        
        Args:
            path_length: Actual path length traveled
            optimal_length: Shortest possible path length
            is_success: Whether the navigation was successful
            
        Returns:
            SPL value (0 if failed, optimal/path ratio if successful)
        """
        return optimal_length / path_length if is_success else 0.0
    
    
    def nav_success(self, pos1: Sequence[float], pos2: Sequence[float], threshold: float = 0.8) -> bool:
        """Determine if navigation was successful based on position threshold.
        
        Args:
            pos1: Current position
            pos2: Target position
            threshold: Maximum allowed distance for success
            
        Returns:
            True if distance is below threshold, False otherwise
        """
        return self.NE(pos1, pos2) < threshold
    
    
    def SR(self, errors: List[float], threshold: float = 0.8) -> float:
        """Calculate Success Rate (SR) from list of navigation errors.
        
        Args:
            errors: List of navigation error values
            threshold: Success distance threshold
            
        Returns:
            Ratio of successful attempts (errors < threshold)
        """
        return sum(e < threshold for e in errors) / len(errors) if errors else 0.0