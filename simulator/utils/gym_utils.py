from gym import spaces
import gym
from typing import List

def flatten_tuple_spaces(tuple_spaces: List[spaces.Tuple]) -> spaces.Tuple:
    flat_spaces = []
    for tup_space in tuple_spaces:
        assert isinstance(tup_space, spaces.Tuple), f"Expected Tuple space, got {type(tup_space)}"
        flat_spaces.extend(tup_space.spaces)
    return spaces.Tuple(flat_spaces)
