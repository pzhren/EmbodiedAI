import math
import os
import pickle
import numpy as np
import heapq

from matplotlib import pyplot as plt
from matplotlib.patches import Polygon


def diagonal_distance(start, end):  # Chebyshev distance
    return max(abs(start[0] - end[0]), abs(start[1] - end[1]))


def manhattan_distance(start, end):  # Manhattan distance
    return abs(start[0] - end[0]) + abs(start[1] - end[1])


def euclidean_distance(start, end):  # Euclidean distance
    return math.hypot(start[0] - end[0], start[1] - end[1])


def heuristic(start, end, name='euclidean'):
    heuristics = {
        'diagonal': diagonal_distance,
        'euclidean': euclidean_distance,
        'manhattan': manhattan_distance
    }
    if name not in heuristics:
        raise Exception(f'Error heuristic name: {name}!')
    return heuristics[name](start, end)


class Priority:
    '''
    Priority class, compare k1 first, then k2
    '''
    def __init__(self, k1, k2):
        self.k1 = k1
        self.k2 = k2

    def __lt__(self, other):  # Define < operation
        return self.k1 < other.k1 or (self.k1 == other.k1 and self.k2 < other.k2)

    def __le__(self, other):  # Define <= operation
        return self.k1 < other.k1 or (self.k1 == other.k1 and self.k2 <= other.k2)


class Node:
    '''
    Node class, compare node priority
    '''
    def __init__(self, pos: (int, int), priority: Priority):
        self.pos = pos  # X Y
        self.priority = priority

    def __lt__(self, other):  # Define < operation
        return self.priority < other.priority

    def __le__(self, other):  # Define <= operation
        return self.priority <= other.priority


class PriorityQueue:
    '''
    Optimized priority queue
    '''
    def __init__(self):
        self.queue = []  # Node queue
        self.nodes = []  # Node position list

    def is_empty(self):
        # Check if queue is empty
        return len(self.queue) == 0

    def top(self):
        return self.queue[0].pos

    def top_key(self):
        if self.is_empty():
            return Priority(float('inf'), float('inf'))
        return self.queue[0].priority

    def pop(self):
        # Remove the first node
        node = heapq.heappop(self.queue)
        self.nodes.remove(node.pos)
        return node

    def insert(self, pos, priority):
        # Create and add node
        node = Node(pos, priority)
        heapq.heappush(self.queue, node)
        self.nodes.append(pos)

    def remove(self, pos):
        # Remove specified node and reorder
        self.queue = [n for n in self.queue if n.pos != pos]
        heapq.heapify(self.queue)  # Reorder
        self.nodes.remove(pos)

    def update(self, pos, priority):
        # Update priority value of specified position
        for n in self.queue:
            if n.pos == pos:
                n.priority = priority
                break


class DStarLite:
    def __init__(self,
                 map: np.ndarray,
                 area_range,  # [x_min, x_max, y_min, y_max] actual coordinate range
                 scale_ratio=5,  # Map scale ratio
                 resolution=0.06,
                 react_radius=200, # Reaction radius
                 vision_radius=math.pi / 3,  # Vision radius
                 max_path_length=2000  # Maximum path length
                 ):

        self.map = map
        self.background = map.copy()
        self.X = map.shape[0]
        self.Y = map.shape[1]
        (self.x_min, self.x_max, self.y_min, self.y_max) = area_range
        self.scale_ratio = scale_ratio
        self.resolution = resolution
        self.react_radius = math.floor(react_radius / scale_ratio)
        self.vision_radius = vision_radius
        self.max_path_length = max_path_length

        self.dyna_obs_list = []  # Dynamic obstacle position list [(x, y)]
        self.dyna_obs_occupy = []  # Dynamic obstacle occupied position list

        # free:0, obs:1, dyna_obs:2
        self.idx_to_object = {
            0: "free",
            1: "obstacle",
            2: "unseen"
        }
        self.object_to_idx = dict(zip(self.idx_to_object.values(), self.idx_to_object.keys()))
        self.object_to_cost = {
            "free": 0,
            "obstacle": float('inf'),
            "unseen": float('inf')
        }

        self.cost_map = None
        self.cost_background = None

        self.s_start = None  # (int,int) Must be a tuple (tuples can be used directly as matrix indices)
        self.s_goal = None  # (int,int)
        self.s_last = None  # (int,int)
        self.U = PriorityQueue()
        self.k_m = 0
        self.rhs = np.ones((self.X, self.Y)) * np.inf
        self.g = self.rhs.copy()
        self.path = []


    def reset(self):
        '''
        (After completing a navigation)
        Reset 1. Environment variables
                2. DStarLite variables
        '''
        # env reset
        self.map = self.background.copy()
        self.cost_map = self.cost_background.copy()
        self.dyna_obs_list.clear()
        self.dyna_obs_occupy.clear()
        self.path.clear()
        # dstar_lite reset
        self.s_start = None
        self.s_goal = None
        self.s_last = None
        self.U = PriorityQueue()
        self.k_m = 0
        self.rhs = np.ones((self.X, self.Y)) * np.inf
        self.g = self.rhs.copy()


    def calculate_key(self, s: (int, int)):
        '''
        Calculate key1, key2 of position s
        :returns:
            Priority(k1, k2): Comparable object
        '''
        k1 = min(self.g[s], self.rhs[s]) + heuristic(self.s_start, s) + self.k_m
        k2 = min(self.g[s], self.rhs[s])
        return Priority(k1, k2)


    def c(self, u: (int, int), v: (int, int), c_old=None) -> float:
        '''
        Calculate path cost and target position cost between nodes (target position cost is 0 when using path cost)
        (Since the expansion direction is from the end point to the start point, v is the node, and u is the neighbor expanded by v)
        Args:
            u:     from pos
            v:     to pos
            c_old: Old cost of node v
        '''
        if u == v:
            return 0.0
        if c_old is not None:
            obj_cost = c_old
        else:
            obj_cost = self.cost_map[v]
        if obj_cost > 0:
            return obj_cost
        return heuristic(u, v)

    def contain(self, u: (int, int)):
        '''
        Check if node u is in the queue
        '''
        return u in self.U.nodes

    def update_vertex(self, u: (int, int)):
        '''
        Determine node status, update queue
            Inconsistent and in queue   -->  Update key
            Inconsistent and not in queue -->  Calculate key and add to queue
            Consistent and in queue     -->  Remove from queue
        '''
        if self.g[u] != self.rhs[u] and self.contain(u):
            self.U.update(u, self.calculate_key(u))
        elif self.g[u] != self.rhs[u] and not self.contain(u):
            self.U.insert(u, self.calculate_key(u))
        elif self.g[u] == self.rhs[u] and self.contain(u):
            self.U.remove(u)


    def compute_shortest_path(self):
        '''
        Compute the shortest path
        '''

        c_map = self.map.copy()

        while self.U.top_key() < self.calculate_key(self.s_start) or self.rhs[self.s_start] > self.g[self.s_start]:
            u = self.U.top()
            c_map[u] = 10
            k_old = self.U.top_key()
            k_new = self.calculate_key(u)
            if k_old < k_new:
                self.U.update(u, k_new)
            elif self.g[u] > self.rhs[u]:  # Over-consistent
                self.g[u] = self.rhs[u]
                self.U.remove(u)
                pred = self.get_neighbors(u)
                for s in pred:
                    if s != self.s_goal:
                        self.rhs[s] = min(self.rhs[s], self.c(s, u) + self.g[u])
                    self.update_vertex(s)
            else:  # Under-consistent
                g_old = self.g[u]
                self.g[u] = float('inf')
                pred = self.get_neighbors(u)
                for s in pred + [u]:
                    if self.rhs[s] == self.c(s, u) + g_old:
                        if s != self.s_goal:
                            succ = self.get_neighbors(s)
                            self.rhs[s] = min([self.c(s, s_) + self.g[s_] for s_ in succ])
                    self.update_vertex(s)


    def _planning(self, s_start, s_goal, debug=False):
        '''
        Plan path (actual implementation)
        Args:
            dyna_obs: Dynamic obstacle position list
            step_num: Number of steps to move
        '''
        # Ensure goal is valid
        if not self.in_bounds_without_obstacle(s_goal):
            s_goal = self.validate_pos(s_goal)
        # First planning needs to initialize rhs and add goal to queue, compute shortest path
        if self.s_goal is None:
            self.s_start = tuple(s_start)
            self.s_goal = tuple(s_goal)
            self.s_last = tuple(s_start)
            self.rhs[tuple(s_goal)] = 0
            self.U.insert(tuple(s_goal), Priority(k1=heuristic(tuple(s_start), tuple(s_goal)), k2=0))

            self.compute_shortest_path()  # Compute shortest path
            self.path = self.get_path()
            return self.path
        # Subsequent planning only updates the start point, directly use the original path (remove the part that has been walked)
        else:
            self.s_start = tuple(s_start)
            return self.path


    def planning(self, s_start, s_goal, debug=False):
        '''
        Path planning 
        (for external use, handle conversion between actual coordinates and map coordinates)
        '''
        # Actual coordinates -> Map coordinates
        s_start = self.real2map(s_start)
        if self.s_goal is None:
            s_goal = self.real2map(s_goal)
        else:
            s_goal = self.s_goal

        self._planning(s_start, s_goal, debug)

       # Map coordinates -> Actual coordinates
        path = [self.map2real(node) for node in self.path]
        return path


    def get_path(self):
        '''
        Get path
        Args:
            step_num: Number of steps
        return:
            path: [(x, y), ...]
        '''
        if self.s_start is None or self.s_goal == self.s_start:
            return []
        path = []
        cur = self.s_start
        for i in range(self.max_path_length):
            succ = [s_ for s_ in self.get_neighbors(cur)]
            cur = succ[np.argmin([self.c(cur, s_) + self.g[s_] + 20 * (s_ in path) for s_ in succ])]  # Avoid jitter (there will be a penalty for walking on repeated points)
            path.append(cur)
            if cur == self.s_goal:
                while cur in self.dyna_obs_occupy:  # Ensure the goal is not within the range of dyna_obs
                    cur = path.pop()
                break
        return path


    def in_bounds_without_obstacle(self, pos):
        '''
        Check if position is within map range and not a static obstacle
        '''
        (x, y) = pos
        return 0 <= x < self.X and 0 <= y < self.Y and self.map[x, y] != self.object_to_idx["obstacle"] \
                    and self.map[x, y] != self.object_to_idx["unseen"]


    def get_neighbors(self, pos, mode=8):
        '''
        Get neighboring nodes, within map range
        '''
        (x_, y_) = pos
        neighbors = [(x_ + 1, y_), (x_ - 1, y_), (x_, y_ + 1), (x_, y_ - 1), (x_ + 1, y_ + 1), (x_ - 1, y_ + 1),
                     (x_ + 1, y_ - 1), (x_ - 1, y_ - 1)]
        neighbors = filter(self.in_bounds_without_obstacle, neighbors)  # Ensure position is within map range and not a static obstacle
        return list(neighbors)


    def compute_cost_map(self):
        # Compute cost_map of the current map
        self.cost_map = np.zeros_like(self.map, dtype=float)
        for idx, obj in self.idx_to_object.items():
            self.cost_map[self.map == idx] = self.object_to_cost[obj]

        # Expand the influence range of static obstacles
        obs_pos = np.where(self.map == self.object_to_idx['obstacle'])  # Static obstacle position list
        for (x, y) in zip(obs_pos[0], obs_pos[1]):
            start_x, end_x = max(x - 1, 0), min(x + 1, self.X - 1)
            start_y, end_y = max(y - 1, 0), min(y + 1, self.Y - 1)
            for cost in range(28, 0, -4):
                for x_ in range(start_x, end_x + 1):
                    self.cost_map[x_, start_y] = max(self.cost_map[x_, start_y], cost)
                for y_ in range(start_y + 1, end_y + 1):
                    self.cost_map[end_x, y_] = max(self.cost_map[end_x, y_], cost)
                for x_ in range(end_x - 1, start_x - 1, -1):
                    self.cost_map[x_, end_y] = max(self.cost_map[x_, end_y], cost)
                for y_ in range(end_y - 1, start_y, -1):
                    self.cost_map[start_x, y_] = max(self.cost_map[start_x, y_], cost)
                start_x, end_x = max(start_x - 1, 0), min(end_x + 1, self.X - 1)
                start_y, end_y = max(start_y - 1, 0), min(end_y + 1, self.Y - 1)

        self.cost_background = self.cost_map.copy()


    def update_map(self, u_map, u_range): 
        '''
        Dynamically update the map
        '''
        self.map = self.background.copy()  # Reset map
        [u_x_min, u_x_max, u_y_min, u_y_max] = u_range
        if u_x_min < self.x_min or u_x_max > self.x_max or u_y_min < self.y_min or u_y_max > self.y_max:
            print("out of range")
            return 0
        (u_X_min, u_Y_min) = self.real2map((u_x_min, u_y_min))
        (u_X_max, u_Y_max) = self.real2map((u_x_max, u_y_max))
        u_X = u_map.shape[0]
        u_Y = u_map.shape[1]
        for ix in range(u_X):
            for iy in range(u_Y):
                if u_map[ix][iy] == 1:  # Update obstacles in the map based on u_map
                    self.map[u_X_min+ix][u_Y_min+iy] = 1

        # 更新cost_map
        self.cost_map = np.zeros_like(self.map, dtype=float)
        for idx, obj in self.idx_to_object.items():
            self.cost_map[self.map == idx] = self.object_to_cost[obj]

        # Update cost_map
        obs_pos = np.where(self.map == self.object_to_idx['obstacle'])  # Static obstacle position list
        for (x, y) in zip(obs_pos[0], obs_pos[1]):
            start_x, end_x = max(x - 1, 0), min(x + 1, self.X - 1)
            start_y, end_y = max(y - 1, 0), min(y + 1, self.Y - 1)
            for cost in range(28, 0, -4):
                for x_ in range(start_x, end_x + 1):
                    self.cost_map[x_, start_y] = max(self.cost_map[x_, start_y], cost)
                for y_ in range(start_y + 1, end_y + 1):
                    self.cost_map[end_x, y_] = max(self.cost_map[end_x, y_], cost)
                for x_ in range(end_x - 1, start_x - 1, -1):
                    self.cost_map[x_, end_y] = max(self.cost_map[x_, end_y], cost)
                for y_ in range(end_y - 1, start_y, -1):
                    self.cost_map[start_x, y_] = max(self.cost_map[start_x, y_], cost)
                start_x, end_x = max(start_x - 1, 0), min(end_x + 1, self.X - 1)
                start_y, end_y = max(start_y - 1, 0), min(end_y + 1, self.Y - 1)


    def map2real(self, pos):
        '''
        Map coordinates -> Actual coordinates
        '''
        x = pos[0] * self.scale_ratio * self.resolution + self.x_min
        y = pos[1] * self.scale_ratio * self.resolution + self.y_min
        return tuple((x, y))


    def real2map(self, pos, reachable_assurance=True):
        '''
        Actual coordinates -> Map coordinates, 
        reachable_assurance: Ensure the point is not on a static obstacle
        '''
        x = round((pos[0] - self.x_min) / (self.scale_ratio * self.resolution))
        y = round((pos[1] - self.y_min) / (self.scale_ratio * self.resolution))
        # Ensure the point is not on a static obstacle, otherwise keep expanding outward until a non-static obstacle position is found
        if reachable_assurance:
            return self.validate_pos((x, y))
        else:
            return tuple((x, y))


    def validate_pos(self, pos):
        '''
        For invalid pos, find the nearest valid coordinate around
        '''
        (x, y) = pos
        x = max(0, min(x, self.X - 1))
        y = max(0, min(y, self.Y - 1))
        if self.cost_map[x, y] != 0:
            start_x, end_x = max(x - 1, 0), min(x + 1, self.X - 1)
            start_y, end_y = max(y - 1, 0), min(y + 1, self.Y - 1)
            while True:
                for x_ in range(start_x, end_x + 1):
                    if self.cost_map[x_, start_y] == 0:
                        return tuple((x_, start_y))
                for y_ in range(start_y + 1, end_y + 1):
                    if self.cost_map[end_x, y_] == 0:
                        return tuple((end_x, y_))
                for x_ in range(end_x - 1, start_x - 1, -1):
                    if self.cost_map[x_, end_y] == 0:
                        return tuple((x_, end_y))
                for y_ in range(end_y - 1, start_y, -1):
                    if self.cost_map[start_x, y_] == 0:
                        return tuple((start_x, y_))
                start_x, end_x = max(start_x - 1, 0), min(end_x + 1, self.X - 1)
                start_y, end_y = max(start_y - 1, 0), min(end_y + 1, self.Y - 1)
        return tuple((x, y))

    def draw_graph(self, step_num, yaw):
        '''
        Args:
            step_num: Number of steps to move
            yaw:      Robot orientation (radians)
        '''
        # Scale coordinate offset
        offset = (self.x_min / self.scale_ratio, self.x_max / self.scale_ratio,
                  self.y_min / self.scale_ratio, self.y_max / self.scale_ratio)
        # Draw start and goal
        if self.s_start:
            start = (self.s_start[0] + offset[0], self.s_start[1] + offset[2])
            plt.plot(start[1], start[0], 'x', color='r')
        if self.s_goal:
            goal = (self.s_goal[0] + offset[0], self.s_goal[1] + offset[2])
            plt.plot(goal[1], goal[0], 'x', color='darkorange')

        # Draw map: X rows Y columns, the first row is at the bottom
        # Range: Horizontal Y[-80,290] Vertical X[-70,120]
        plt.imshow(self.map, cmap='binary', alpha=0.5, origin='lower',
                   extent=(offset[2], offset[3],
                           offset[0], offset[1]))

        # Draw search path
        plt.plot([y + offset[2] for (x, y) in self.path],
                 [x + offset[0] for (x, y) in self.path], "-g")

        if self.s_start:
            # Draw movement path
            next_step = min(step_num, len(self.path))
            plt.plot([start[1], self.path[next_step - 1][1] + offset[2]],
                     [start[0], self.path[next_step - 1][0] + offset[0]], "-r")

            # Draw reaction radius and observation range
            self.plot_circle(start[1], start[0], self.react_radius, 'lightgrey')
            if yaw is not None:
                plt.plot([start[1], start[1] + self.react_radius * (math.sin(yaw + self.vision_radius))],
                         [start[0], start[0] + self.react_radius * (math.cos(yaw + self.vision_radius))], "aqua",
                         linewidth=1)
                plt.plot([start[1], start[1] + self.react_radius * (math.sin(yaw - self.vision_radius))],
                         [start[0], start[0] + self.react_radius * (math.cos(yaw - self.vision_radius))], "aqua",
                         linewidth=1)

        plt.xlabel('y', loc='right')
        plt.ylabel('x', loc='top')
        plt.grid(color='black', linestyle='-', linewidth=0.5)


    def draw_rhs(self, rhs):
        '''
        Draw map: X rows Y columns, the first row is at the bottom
        '''

        # Scale coordinate offset
        offset = (self.x_min / self.scale_ratio, self.x_max / self.scale_ratio,
                  self.y_min / self.scale_ratio, self.y_max / self.scale_ratio)

        # Replace infinite values with np.nan
        rhs = np.ma.masked_invalid(rhs)

        # Set color mapping range
        vmin = np.nanmin(rhs)
        vmax = np.nanmax(rhs)

        # Create a figure object
        fig, ax = plt.subplots()

        # Draw heatmap
        heatmap = ax.imshow(rhs, cmap='jet', vmin=vmin, vmax=vmax, origin='lower',
                            extent=(offset[2], offset[3],
                                    offset[0], offset[1]))
        # Add color bar
        plt.colorbar(heatmap)

        start = (self.s_start[0] + offset[0], self.s_start[1] + offset[2])
        goal = (self.s_goal[0] + offset[0], self.s_goal[1] + offset[2])

        # Draw start and goal
        plt.plot(start[1], start[0], 'x', color='black')
        plt.plot(goal[1], goal[0], 'x', color='darkorange')

        # Set figure title and axis labels
        ax.set_title('Heatmap(G)')
        ax.set_xlabel('y', loc='right')
        ax.set_ylabel('x', loc='top')
        ax.grid(color='black', linestyle='-', linewidth=0.5)
        plt.show()

    @staticmethod
    def plot_circle(y, x, size, color="lightgrey"):  # pragma: no cover
        '''
            Draw a circle with (x,y) as the center and size as the radius
        '''
        deg = list(range(0, 360, 5))
        deg.append(0)
        yl = [y + size * math.cos(np.deg2rad(d)) for d in deg]
        xl = [x + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(yl, xl, color)

    def triangle_occupy(self, obs, occupy_radius):
        '''
            Calculate the three vertices of an equilateral triangle and return the triangle object
        Args:
            obs:           Midpoint of the base of the triangle
            occupy_radius: Height of the triangle
        '''
        dist = euclidean_distance(self.s_start, obs)
         # Coordinates of obs (midpoint of the base)
        x1, y1 = obs
        # Coordinates of vertex A
        x2 = x1 + occupy_radius * ((self.s_start[0] - x1) / dist)
        y2 = y1 + occupy_radius * ((self.s_start[1] - y1) / dist)

        # Calculate the coordinates of vector AP
        AP_x = x1 - x2
        AP_y = y1 - y2
       # Calculate the coordinates of vector AB
        AB_x = AP_x * math.cos(math.pi / 6) - AP_y * math.sin(math.pi / 6)
        AB_y = AP_x * math.sin(math.pi / 6) + AP_y * math.cos(math.pi / 6)
        # Calculate the coordinates of vector AC
        AC_x = AP_x * math.cos(math.pi / 6) + AP_y * math.sin(math.pi / 6)
        AC_y = -AP_x * math.sin(math.pi / 6) + AP_y * math.cos(math.pi / 6)

        # Calculate the coordinates of vertex B
        x3 = x2 + AB_x
        y3 = y2 + AB_y
        # Calculate the coordinates of vertex C
        x4 = x2 + AC_x
        y4 = y2 + AC_y

        ver1 = (x2, y2)
        ver2 = (x3, y3)
        ver3 = (x4, y4)

        return Polygon([ver1, ver2, ver3], closed=True, fill=True)
