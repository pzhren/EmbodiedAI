from navigate_python import map_show,navigate,dstar_lite,discretize_map
from navigate_python.map_show import *
from navigate_python.navigate import *
from navigate_python.dstar_lite import *
from navigate_python.discretize_map import *
def read_json():
    import json

    scene_root_path = "/data1/lfwj/hssd_scenes/final_selected_usd/"
    # 根据json文件读取物品ID
    with open('config.json', 'r', encoding='utf-8') as file:
        data = json.load(file)

    # 提取目标物品的ID
    target_ids = data['Scene']
    scene_ID = scene_root_path + target_ids + "/" + target_ids+".usd"
    print(target_ids,scene_ID)

#计算两个位置之间是否有墙体阻隔,考虑map中的墙体为灰色数值2,如果两者连线中包含灰色数值2,则表示穿墙，失败
def is_success(img_path,obj_world_pos,robot_world_pos):
    hm = HeightMap(img_path) # 定义地图类
    xy_range = hm.compute_range()
    hm.make_map()
    hm_map = hm.get_map()
    navigator = Navigator(area_range=xy_range, map=hm_map, scale_ratio=1) # 定义导航类
    navigator.planner.compute_cost_map()
    # 获取物体和机器地图坐标
    obj_map_pos = navigator.planner.real2map(obj_world_pos,reachable_assurance=False)
    robot_map_pos = navigator.planner.real2map(robot_world_pos,reachable_assurance=False)

    # 计算地图上两者连线是否经过灰色区域
    x0, y0 = obj_map_pos
    x1, y1 = robot_map_pos

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0 # 起点
    sx = -1 if x0 > x1 else 1 # 递增步长
    sy = -1 if y0 > y1 else 1

    if dx > dy: # 确定主轴
        err = dx / 2.0
        while x != x1:
            if hm_map[x][y] == 2:  # 检查是否为灰色区域
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
                return False
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    return True

def validate_pos4grasp(self, pos):
    '''
        对于不合法的pos,找到周围距离最近的合法坐标,可以让机器人不要穿墙去抓取
    '''
    (x, y) = pos
    x = max(0, min(x, self.X - 1)) # 使坐标在地图范围内
    y = max(0, min(y, self.Y - 1))
    # if self.idx_to_object[self.map[x, y]] == 'obstacle':
    if self.cost_map[x, y] != 0: # 如果这个点不为0，即不可行驶区域
        start_x, end_x = max(x - 1, 0), min(x + 1, self.X - 1)
        start_y, end_y = max(y - 1, 0), min(y + 1, self.Y - 1)
        center = (x, y)
        while True:
            for p in sort_points_by_distance_to_center(start_x, end_x, start_y, end_y, center):
                if self.cost_map[p[0], p[1]] == 0: # 这里加多一个判断条件，如果两者连线中间有灰色区域点，就继续筛选
                    is_no_grey = True
                    dx = abs(p[0] - x)
                    dy = abs(p[1] - y)
                    x_, y_ = x, y # 起点
                    sx = -1 if x > p[0] else 1 # 递增步长
                    sy = -1 if y > p[1] else 1

                    if dx > dy: # 确定主轴
                        err = dx / 2.0
                        while x_ != p[0]:
                            print("checking",x,y)
                            print("hm_map[x][y]",self.map[x][y])
                            if self.map[x_][y_] == 2:  # 检查是否为灰色区域
                                print("fail,x,y",x_,y_)  
                                is_no_grey=  False
                            err -= dy
                            if err < 0:
                                y_ += sy
                                err += dx
                            x_ += sx
                    else:
                        err = dy / 2.0
                        while y_ != p[1]:
                            if self.map[x_][y_] == 2:
                                print("fail,x,y",x_,y_)  
                                is_no_grey=  False
                            err -= dx
                            if err < 0:
                                x_ += sx
                                err += dy
                            y_ += sy
                    if is_no_grey:
                        return p
                    
            start_x, end_x = max(start_x - 1, 0), min(end_x + 1, self.X - 1)
            start_y, end_y = max(start_y - 1, 0), min(end_y + 1, self.Y - 1)
    return tuple((x, y))

def init_robot_pos(img_path):
    '''
    从地图中随即选择一个位置，作为机器人的初始位置
    '''
    hm = HeightMap(img_path) # 定义地图类
    xy_range = hm.compute_range()
    hm.make_map()
    hm_map = hm.get_map()
    navigator = Navigator(area_range=xy_range, map=hm_map, scale_ratio=1) # 定义导航类
    navigator.planner.compute_cost_map()
    # 随即获取机器人的初始位置
    import random
    random_x = random.randint(hm.X, hm.X + hm.height * hm.resolution)
    random_y = random.randint(hm.Y, hm.Y + hm.width * hm.resolution)
    robot_map_pos = navigator.planner.real2map([random_x,random_y],reachable_assurance=False)
    return robot_map_pos

import pandas as pd

def append_to_csv(file_path, json_file_path, is_success, path_length, NE):
    # 创建新行的数据字典
    new_data = pd.DataFrame([{
        "json_file_path": json_file_path,
        "is_success": is_success,
        "path_length": path_length,
        "NE": NE
    }])
    
    # 读取 CSV 文件，如果不存在则创建一个新的 DataFrame
    try:
        # 读取 CSV 文件
        df = pd.read_csv(file_path)
        
        # 使用 pd.concat 追加新数据
        df = pd.concat([df, new_data], ignore_index=True)
        
        # 将更新后的 DataFrame 写入 CSV 文件
        df.to_csv(file_path, index=False)
        print("成功添加数据到文件中：", file_path)
    except FileNotFoundError:
        # 文件不存在时，创建新文件并写入数据
        new_data.to_csv(file_path, index=False)
        print("文件不存在，已创建新文件并写入数据：", file_path)


import os


def create_imgs_folder(json_file_path):
    json_file_dir = os.path.dirname(json_file_path)
    parent_folder = os.path.dirname(json_file_dir)
    folder_number = os.path.basename(json_file_dir)

    new_folder_name = f"{folder_number}_imgs"
    new_folder_path = os.path.join(parent_folder, new_folder_name)

    os.makedirs(new_folder_path, exist_ok=True)

    return new_folder_path
