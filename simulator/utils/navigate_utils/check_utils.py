from .map_show import *
from .navigate import *
from .dstar_lite import *
from .discretize_map import *
import random
import math
import pandas as pd
import os

def check_wall_block(hm, navigator ,obj_world_pos,robot_world_pos,scene_offset=None):
    '''
    Check if there are any walls blocking the path between two positions.
    Walls are represented by a gray value of 2 in the map.
    If the line connecting the two points passes through a gray value of 2, it indicates a wall, and the check fails.
    '''
    # hm = HeightMap(img_path) # Define the map class
    xy_range = hm.compute_range()
    hm.make_map()
    hm_map = hm.get_map()
    # navigator = Navigator(area_range=xy_range, map=hm_map, scale_ratio=1)  # Define the navigator class
    navigator.planner.compute_cost_map()

    # Get the map coordinates of the object and the robot
    obj_map_pos = navigator.planner.real2map(obj_world_pos,reachable_assurance=False)
    robot_map_pos = navigator.planner.real2map(robot_world_pos,reachable_assurance=False)
    # print("obj_map_pos",obj_map_pos)
    # print("robot_map_pos",robot_map_pos)
    # Calculate whether the line connecting the two points on the map passes through a gray region
    x0, y0 = obj_map_pos
    x1, y1 = robot_map_pos

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
  
    x, y = x0, y0  # Starting point
    sx = -1 if x0 > x1 else 1 # Increment step
    sy = -1 if y0 > y1 else 1

    if dx > dy: # Determine the main axis
        err = dx / 2.0
        while x != x1:
            if hm_map[x][y] == 2:  # Check if it is a gray region
                # print("Wall detected,fail,x,y",x,y)  
                return False
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            if hm_map[x][y] == 2:
                # print("Wall detected,fail,x,y",x,y)  
                return False
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    return True