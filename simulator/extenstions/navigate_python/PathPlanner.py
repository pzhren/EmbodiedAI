from map_show import HeightMap
from navigate import Navigator
from discretize_map import show_map_
import math
import random

class PathPlanner:
    def __init__(self, map_file_path, de_nosing=False):
        """
        Initialize PathPlanner class
        :param map_file_path: Map file path
        :param de_noising: Whether to perform denoising
        """
        self.map_file_path = map_file_path
        self.hm = HeightMap(map_file_path)
        self.xy_range = self.hm.compute_range()
        self.hm.make_map()
        if de_nosing:
            self.hm.de_noising()
        self.hm_map = self.hm.get_map()
        
        self.navigator = Navigator(area_range=self.xy_range, map=self.hm_map, scale_ratio=1)
        self.navigator.planner.compute_cost_map()


    def show_hm_info(self):
        """
        Show height map information
        """
        self.hm.show_info()
    
    
    def show_height_map(self):
        """
        Show height map
        """
        self.hm.map_plot()
        
        
    def show_cost_map(self):
        """
        Show cost map
        """
        show_map_(self.navigator.planner.cost_map)

    def calulate_path_length(self, path):
        """
        Calculate path length
        :param path: Path
        :return: Path length
        """
        if len(path) < 2:
            return 0
        path_length = 0
        for i in range(len(path)-1):
            path_length += math.dist(path[i], path[i+1])
        return path_length

    def navigate_world_coords(self, start, goal, cal_length=False):
        """
        Navigate using world coordinates
        :param start: Start point in world coordinates
        :param goal: Goal point in world coordinates
        :return: Planned path and map path
        """
        path, map_path = self.navigator.navigate(goal, start)
        if cal_length:
            path_len = self.calulate_path_length(path)
        self.show_path(map_path, goal)
        return path, map_path, path_len


    def navigate_map_coords(self, start, goal, cal_length=False):
        """
        Navigate using map coordinates
        :param start: Start point in map coordinates
        :param goal: Goal point in map coordinates
        :return: Planned path and map path
        """
        path, map_path = self.navigator._navigate_(goal, start)
        if cal_length:
            path_len = self.calulate_path_length(path)
        self.show_path(map_path, goal)
        return path, map_path, path_len
    

    def navigate_world_map_coords(self, start, goal, cal_length=False):
        """
        Navigate with goal in map coordinates and start in world coordinates
        :param start: Start point in world coordinates
        :param goal: Goal point in map coordinates
        :return: Planned path and map path
        """
        path, map_path = self.navigator.navigate_(goal, start)
        if cal_length:
            path_len = self.calulate_path_length(path)
        self.show_path(map_path, goal)
        return path, map_path, path_len


    def show_path(self, map_path, goal):
        """
        Show planned path
        :param map_path: Map path
        :param goal: Goal point in map coordinates
        """
        show_map_(self.navigator.planner.cost_map, map_path, goal)
        
    
    def sample_pos(self, scale=0.5, err=0.1):
        """
        Sample a random drivable position
        :return: Random position
        """
        random_x = random.randint(math.floor((self.hm.X + self.hm.height * self.hm.resolution * (scale - err))), math.floor((self.hm.X + self.hm.height * self.hm.resolution * (scale + err) ) ))
        random_y = random.randint(math.floor((self.hm.Y + self.hm.width * self.hm.resolution * (scale - err))), math.floor((self.hm.Y + self.hm.width * self.hm.resolution  * (scale + err) ) ))
        robot_map_pos = self.navigator.planner.real2map([random_x,random_y])
        robot_real_pos = self.navigator.planner.map2real(robot_map_pos)
        print("Robot initial position",robot_real_pos, robot_map_pos)
        return robot_real_pos
    

# if __name__ == '__main__':
#     map_file_path = './occupancy_map/w_350h_453r_0.050000X_-20.575000381469728Y_-4.875000095367431.png'
#     planner = PathPlanner(map_file_path)
#     planner.show_hm_info()
#     planner.show_height_map()
#     planner.show_cost_map()
#     print(planner.sample_pos())
    
#     # Example: Navigate using map coordinates
#     pos_world = [34, 68]
#     goal_world = [120, 100]
#     # goal_world = [34, 70]
#     path_world, map_path_world, path_len = planner.navigate_map_coords(pos_world, goal_world, cal_length=True)
#     print("World Coordinates Path:", path_world)
#     print("Map Coordinates Path:", map_path_world)
#     print("Path Length:", path_len)
