import os
import pickle
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import re


def read_bin(file):
    if os.path.exists(file):
        raw_map = np.loadtxt(file)
    else:
        print("file doesn't exit!")
        return 0
    return raw_map


class HeightMap:
    def __init__(self, file):
        self.file = file
        self.raw_map = Image.open(self.file).convert('RGB')
        self.map = None
        split_file = re.split('w_|h_|r_|X_|Y_|.png', self.file)
        self.width = float(split_file[1])           # Height map width
        self.height = float(split_file[2])          # Height map height
        self.resolution = float(split_file[3])      # Map resolution
        self.X = float(split_file[4])               # X offset of height map origin relative to world coordinates
        self.Y = float(split_file[5])               # Y offset of height map origin relative to world coordinates
        # Actual coordinate range of the map xmin, xmax, ymin, ymax
        self.area_range = [self.X - self.width * self.resolution, self.X + self.width * self.resolution,
                           self.Y - self.height * self.resolution, self.Y + self.height * self.resolution]


    def make_map(self):
        image_array = np.array(self.raw_map)
        self.map = np.zeros((int(self.height), int(self.width)), dtype=int)
        for i in range(image_array.shape[0]):
            for j in range(image_array.shape[1]):
                # Get pixel value
                pixel = image_array[i, j]
                # Check if it is a black pixel (1 represents an obstacle)
                if np.array_equal(pixel, [0, 0, 0]):
                    self.map[i, j] = 1
                # Check if it is a white pixel (0 represents walkable)
                elif np.array_equal(pixel, [255, 255, 255]):
                    self.map[i, j] = 0
                else:   # (2 represents unobserved)
                    self.map[i, j] = 2
        return self.map


    def de_noising(self):
        if self.map is None:
            print("please reconstruct the map first!")
            return 0
        noise_list = np.argwhere(self.map == 2)    # Get coordinates of all unobserved points in the height map
        for coord in noise_list:
            if coord[0]*coord[1] == 0 or coord[1] == self.width-1 or coord[0] == self.height-1:
                continue                            # Do not process edge points of the map
            else:                                   # Get values around the unobserved point
                try:
                    around = np.array([self.map[coord[0]-1][coord[1]-1], self.map[coord[0]][coord[1]-1],
                                    self.map[coord[0]+1][coord[1]-1], self.map[coord[0]-1][coord[1]],
                                    self.map[coord[0]+1][coord[1]], self.map[coord[0]-1][coord[1]+1],
                                    self.map[coord[0]][coord[1]+1], self.map[coord[0]+1][coord[1]+1]])
                except IndexError:
                    print("indexerror", coord)
                    print("self.width", self.width)
                    print("self.height", self.height)
                if np.count_nonzero(around == 1) > 4:   # There are obstacles around the unobserved point
                    self.map[coord[0]][coord[1]] = 1
                elif np.count_nonzero(around == 0) > 4:    # There are more than or equal to 4 walkable grids around the unobserved point
                    self.map[coord[0]][coord[1]] = 0


    def map_plot(self):
        if self.map is None:
            print("please reconstruct the map first!")
            return 0
        fig, ax = plt.subplots()
        im = ax.imshow(self.map, cmap='hot_r', interpolation='nearest')
        ax.set_xticks(np.arange(self.map.shape[1]))
        ax.set_yticks(np.arange(self.map.shape[0]))
        ax.set_xticklabels(np.arange(1, self.map.shape[1] + 1))
        ax.set_yticklabels(np.arange(1, self.map.shape[0] + 1))
        plt.colorbar(im)
        plt.show()


    def show_info(self):
        print("width: %f, height: %f" % (self.width, self.height))
        print("resolution: %f" % self.resolution)
        print("X: %f, Y: %f" % (self.X, self.Y))


    def save_pkl(self, k):
        if self.map is None:
            print("please reconstruct the map first!")
            return 0
        file_name = f'height_map{k}.pkl'
        if not os.path.exists(file_name):
            open(file_name, 'w').close()
        with open(file_name, 'wb') as file:
            pickle.dump(self.map, file)


    def compute_range(self):
        min_x = self.X
        max_x = self.X + self.height*self.resolution
        min_y = self.Y
        max_y = self.Y + self.width*self.resolution
        # print("min x: %f, max x: %f" % (min_x, max_x))
        # print("min y: %f, max y: %f" % (min_y, max_y))
        return [min_x, max_x, min_y, max_y]


    def get_map(self):
        return self.map


