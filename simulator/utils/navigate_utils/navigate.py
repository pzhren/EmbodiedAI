#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
import math
import os
import pickle
import matplotlib.pyplot as plt
import numpy as np
from .dstar_lite import DStarLite, euclidean_distance


class Navigator:
    '''
    Navigation class
    '''

    def __init__(self,
                 area_range,
                 map,
                 scale_ratio=1,
                 resolution=0.05,
                 react_radius=50,
                 vision_radius=math.pi*3/7,
                 max_iteration=100):
        self.area_range = area_range        # Actual coordinate range of the map xmin, xmax, ymin, ymax
        self.map = map                      # Scaled and discretized map array(X,Y)
        self.scale_ratio = scale_ratio      # Map scale ratio
        self.resolution = resolution
        self.max_iteration = max_iteration  # Maximum planning iteration count

        self.planner = DStarLite(area_range=area_range, map=map, scale_ratio=scale_ratio, 
                                 react_radius=react_radius, resolution=resolution, vision_radius=vision_radius)
        self.yaw = None

    def validate_goal(self, goal):
        '''
        Validate goal
        '''
        return self.planner.map2real(self.planner.real2map(goal))

    def navigate(self, goal: (float, float), pos: (float, float), animation=True):
        '''
        Single navigation until reaching the goal, inputs are in world coordinates
        '''
        goal = np.array(self.validate_goal(goal))  # Validate goal
        pos = np.array(pos)   # Robot's current position and orientation
        self.yaw = None
        # print('------------------navigation_start----------------------')

        path = self.planner.planning(pos, goal)

        self.planner.reset()  # Reset variables after completing a round of navigation
        map_path = [self.planner.real2map(p) for p in path]
        return path, map_path
    
    def navigate_(self, goal: (int, int), pos: (float, float), animation=True):
        '''
        Single navigation until reaching the goal, goal is in map coordinates, pos is in world coordinates
        '''
        goal = np.array(self.planner.map2real(goal))
        pos = np.array(pos)  # Robot's current position and orientation
        self.yaw = None
        # print('------------------navigation_start----------------------')

        path = self.planner.planning(pos, goal)

        self.planner.reset()  # Reset variables after completing a round of navigation
        map_path = [self.planner.real2map(p) for p in path]
        return path, map_path

    def _navigate_(self, goal: (int, int), pos: (int, int), animation=True):
        '''
        Single navigation until reaching the goal, inputs are in map coordinates
        '''
        goal = np.array(self.planner.map2real(goal))
        pos = np.array(self.planner.map2real(pos))  # Robot's current position and orientation
        self.yaw = None
        # print('------------------navigation_start----------------------')

        path = self.planner.planning(pos, goal)

        self.planner.reset()  # Reset variables after completing a round of navigation
        map_path = [self.planner.real2map(p) for p in path]
        return path, map_path


