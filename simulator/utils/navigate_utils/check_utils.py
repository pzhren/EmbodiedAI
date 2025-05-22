from .map_show import *
from .navigate import *
from .dstar_lite import *
from .discretize_map import *
import random
import math
import pandas as pd
import os
from collections import deque
import matplotlib.pyplot as plt
import numpy as np


def check_reachability_with_expansion(hm, navigator , obj_world_pos, robot_world_pos, scene_offset=None, max_expand_steps=10):
    # 地图构建
    hm_map = hm.get_map()
    rows, cols = len(hm_map), len(hm_map[0])

    # 坐标转换
    navigator.planner.compute_cost_map()
    obj_map = navigator.planner.real2map(obj_world_pos, reachable_assurance=False)
    rob_map = navigator.planner.real2map(robot_world_pos, reachable_assurance=False)
    x0, y0 = obj_map
    xR, yR = rob_map

    # Step 1：确保物品在黑色区域，机器人在白色区域
    if hm_map[x0][y0] != 1:
        print("物品不在黑色区域 → 无效地图")
        return False
    if hm_map[xR][yR] != 0:
        print("机器人不在白色区域 → 导航失败")
        return False

    # Step 2：找出与物品相连的黑色区域（Flood Fill）
    black_region = np.zeros((rows, cols), dtype=bool)
    q = deque()
    q.append((x0, y0))
    black_region[x0][y0] = True

    neighbors = [(-1,0),(1,0),(0,-1),(0,1)]
    while q:
        x, y = q.popleft()
        for dx, dy in neighbors:
            nx, ny = x+dx, y+dy
            if 0 <= nx < rows and 0 <= ny < cols:
                if not black_region[nx][ny] and hm_map[nx][ny] == 1:
                    black_region[nx][ny] = True
                    q.append((nx, ny))

    # Step 3：从黑色区域的边缘向白色区域膨胀（限制步数）
    visited = np.zeros((rows, cols), dtype=bool)
    expand_q = deque()

    for x in range(rows):
        for y in range(cols):
            if black_region[x][y]:
                for dx, dy in neighbors:
                    nx, ny = x+dx, y+dy
                    if 0 <= nx < rows and 0 <= ny < cols:
                        if hm_map[nx][ny] == 0 and not visited[nx][ny]:
                            visited[nx][ny] = True
                            expand_q.append((nx, ny, 1))  # 从黑边开始

    robot_found = False
    while expand_q:
        x, y, step = expand_q.popleft()
        if (x, y) == (xR, yR):
            robot_found = True
            break
        if step >= max_expand_steps:
            continue
        for dx, dy in neighbors:
            nx, ny = x+dx, y+dy
            if 0 <= nx < rows and 0 <= ny < cols:
                if not visited[nx][ny] and hm_map[nx][ny] == 0:
                    visited[nx][ny] = True
                    expand_q.append((nx, ny, step+1))

    if robot_found:
        print("机器人在局部膨胀区域内 → 导航成功")
    else:
        print("机器人不在局部膨胀区域内 → 导航失败")

    # visualize_expansion(hm_map, black_region, visited, obj_map, rob_map)
    return robot_found


def visualize_expansion(hm_map, black_region, expansion_region, obj_map, rob_map):
    """
    可视化地图：
    - 灰色：墙
    - 黑色：黑区
    - 白色：白区
    - 绿色：黑色连通区域
    - 蓝色：膨胀区域
    - 红点：机器人
    - 绿点：物品
    """
    rows, cols = hm_map.shape
    color_map = np.zeros((rows, cols, 3), dtype=np.float32)

    for i in range(rows):
        for j in range(cols):
            val = hm_map[i][j]
            if val == 2:
                color_map[i, j] = [0.5, 0.5, 0.5]  # 墙体
            elif val == 1:
                color_map[i, j] = [0, 0, 0]        # 黑区
            else:
                color_map[i, j] = [1, 1, 1]        # 白区

            if black_region[i, j]:
                color_map[i, j] = [0.0, 0.7, 0.0]  # 黑色连通区域用绿色覆盖
            if expansion_region[i, j]:
                color_map[i, j] = [0.6, 0.85, 1.0]  # 膨胀区域 淡蓝

    plt.figure(figsize=(6, 6))
    plt.imshow(color_map, origin='upper')

    # 标注机器人和物品
    x0, y0 = obj_map
    xR, yR = rob_map
    plt.scatter([y0], [x0], c='green', marker='o', s=80, label='Object')
    plt.text(y0 + 0.3, x0, 'O', color='green', fontsize=12)

    plt.scatter([yR], [xR], c='red', marker='o', s=80, label='Robot')
    plt.text(yR + 0.3, xR, 'R', color='red', fontsize=12)

    plt.title('局部膨胀区域可视化')
    plt.axis('off')
    plt.legend()
    plt.show()


