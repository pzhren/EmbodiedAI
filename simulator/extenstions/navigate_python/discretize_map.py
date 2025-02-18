# !/usr/bin/env python3
# -*- encoding: utf-8 -*-


import matplotlib.pyplot as plt
import numpy as np
import pickle
import os

from scipy.ndimage import binary_dilation


def draw_grid_map(grid_map, path=None, goal=None):
    """ 
    Draw a grid map, displaying the path and goal point 
    :param grid_map: 2D grid map 
    :param path: List of path points (optional) 
    :param goal: Coordinates of the goal point (optional) 
    """
    plt.imshow(grid_map, cmap='binary', alpha=0.5, origin='lower')  # Display grid in black and white

    # Draw axis labels
    plt.xlabel('y')
    plt.ylabel('x')

    # Show grid lines
    plt.grid(color='black', linestyle='-', linewidth=0.5)

    # Draw path if path and goal are provided
    if path:
        px, py = zip(*path)  # Unpack path coordinates
        plt.plot(py[:-1], px[:-1]) # Draw path line
        if goal:
            plt.scatter(py[0], px[0], s=10, c='red') # Start point in red, goal point in blue
            plt.scatter(goal[1], goal[0], s=10, c='blue')
        print("path plot")

    plt.show()


def discretize_map(scene, scale_ratio):
    """
    Discretize the scene map into a grid
    :param scene: Scene object
    :param scale_ratio: Discretization scale ratio
    """
    
    X = int(950 / scale_ratio)  # Number of points
    Y = int(1850 / scale_ratio)
    map = np.zeros((X, Y))

    # Fill the grid map, check if each point is reachable
    for x in range(X):
        for y in range(Y):
            # Mark as obstacle if the position is not reachable
            if not scene.reachable_check(x * scale_ratio - 350, y * scale_ratio - 400, Yaw=0):
                map[x, y] = 1
                print(f"Obstacle position: ({x}, {y})")

    # Save the discretized map
    file_name = f'map_{scale_ratio}.pkl'
    if not os.path.exists(file_name):
        open(file_name, 'w').close()
    with open(file_name, 'wb') as file:
        pickle.dump(map, file)
    print('Discretized map saved successfully')


def expand_obstacles(scale_ratio, expand_range=1):
    '''
    Expand obstacle boundaries to generate a dilated map
    :param scale_ratio: Discretization scale ratio
    :param expand_range: Expansion range
    '''
    # Original and dilated map file names
    file_name = f'map_{scale_ratio}.pkl'
    dilated_file_name = f'map_{scale_ratio}_e{expand_range}.pkl'

    if os.path.exists(file_name):
        # Load the original map
        with open(file_name, 'rb') as file:
            map = pickle.load(file)
            
        # Perform dilation on obstacles
        dilated_map = binary_dilation(map, iterations=expand_range)

        # Save the dilated map
        if not os.path.exists(dilated_file_name):
            open(dilated_file_name, 'w').close()
        with open(dilated_file_name, 'wb') as file:
            pickle.dump(dilated_map, file)
        print('Dilated map saved successfully')
    else:
        print(f"Error: File {file_name} does not exist!")


def show_map(file_name, path=None):
    """
    Display the saved map file
    :param file_name: Map file path
    :param path: Optional path parameter
    """
    if os.path.exists(file_name):
        with open(file_name, 'rb') as file:
            map = pickle.load(file)
            draw_grid_map(map, path)
    else:
        print(f"Error: File {file_name} does not exist!")



def show_map_(map, path=None, goal=None):
    """
    Directly display the provided map data
    :param grid_map: 2D grid map
    :param path: Optional path parameter
    :param goal: Optional goal point parameter
    """
    draw_grid_map(map, path, goal)

