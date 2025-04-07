from navigate_python.map_show import *
from navigate_python.navigate import *
from navigate_python.dstar_lite import *
from navigate_python.discretize_map import *
from BaseControl_position import BaseControl
from BaseGrasp_test import BaseGrasp
from transformations import quaternion_from_euler
import random
import math
import pandas as pd
import os

def init_robot_pos(img_path,scene_offset=None):
    '''
    从地图中随即选择一个位置，作为机器人的初始位置
    '''
    hm = HeightMap(img_path,bias=scene_offset) # 定义地图类
    xy_range = hm.compute_range()
    hm.make_map()
    hm_map = hm.get_map()
    navigator = Navigator(area_range=xy_range, map=hm_map, scale_ratio=1) # 定义导航类
    navigator.planner.compute_cost_map()
    # 随即获取机器人的初始位置
    random_x = random.randint(math.floor(hm.X) -1, math.floor(hm.X + hm.height * hm.resolution))
    random_y = random.randint(math.floor(hm.Y) -1, math.floor(hm.Y + hm.width * hm.resolution))
    robot_map_pos = navigator.planner.real2map([random_x,random_y])
    robot_real_pos = navigator.planner.map2real(robot_map_pos)
    print("机器初始位置",robot_real_pos)
    return robot_real_pos

#计算两个位置之间是否有墙体阻隔,考虑map中的墙体为灰色数值2,如果两者连线中包含灰色数值2,则表示穿墙，失败
def is_success(img_path,obj_world_pos,robot_world_pos,scene_offset=None):
    hm = HeightMap(img_path,bias=scene_offset) # 定义地图类
    xy_range = hm.compute_range()
    hm.make_map()
    hm_map = hm.get_map()
    navigator = Navigator(area_range=xy_range, map=hm_map, scale_ratio=1) # 定义导航类
    navigator.planner.compute_cost_map()

    # real_x0, real_y0 = obj_world_pos
    # real_x1, real_y1 = robot_world_pos

    # real_dx = abs(real_x0 - real_x1)
    # real_dy = abs(real_y0 - real_y1)
    # real_dist = math.sqrt(real_dx**2 + real_dy**2)
    # if real_dist > 0.8:
    #     print(real_dist)
    #     print("fail,distance is out of limitation")
    #     return False

    # 获取物体和机器地图坐标
    obj_map_pos = navigator.planner.real2map(obj_world_pos,reachable_assurance=False)
    robot_map_pos = navigator.planner.real2map(robot_world_pos,reachable_assurance=False)
    print("obj_map_pos",obj_map_pos)
    print("robot_map_pos",robot_map_pos)
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
                print("穿墙,fail,x,y",x,y)  
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
                print("穿墙,fail,x,y",x,y)  
                return False
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    return True



#计算两个位置之间是否有墙体阻隔,考虑map中的墙体为灰色数值2,如果两者连线中包含灰色数值2,则表示穿墙，失败
def is_success2(img_path,obj_world_pos,robot_world_pos,next_obj_range,scene_offset=None):
    hm = HeightMap(img_path,bias=scene_offset) # 定义地图类
    xy_range = hm.compute_range()
    hm.make_map()
    hm_map = hm.get_map()
    navigator = Navigator(area_range=xy_range, map=hm_map, scale_ratio=1) # 定义导航类
    navigator.planner.compute_cost_map()

    # 获取物体地图坐标
    obj_map_pos = navigator.planner.real2map(obj_world_pos,reachable_assurance=False)

    # 获取桌子的范围
    minx,miny,maxx,maxy = next_obj_range

    # 计算地图上两者连线是否经过灰色区域
    robot_map_pos = navigator.planner.real2map(robot_world_pos,reachable_assurance=False)
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
                print("fail,x,y",x,y)  
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
                print("fail,x,y",x,y)  
                return False
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    
    print("不经过灰色区域，判断是否在桌子上")

    # 最后判断是否在桌子平面上
    if minx <= obj_world_pos[0] <= maxx and  miny <= obj_world_pos[1] <= maxy:
        print("obj is in the area")
        return True
    else:
        print("不在范围内,x:",minx,maxx,"y:",miny,maxy)
        print("物品的真实世界坐标",obj_world_pos)
        return False



# 计算一串路径点的总距离
def calculate_path_length(path):
    total_distance = 0.0
    
    # 遍历每一对相邻点
    for i in range(1, len(path)):
        # 获取当前点和上一个点的坐标
        x1, y1 = path[i - 1]
        x2, y2 = path[i]
        
        # 计算欧几里得距离
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        
        # 累加距离
        total_distance += distance
    
    return total_distance



# 将结果加入csv文件中，导航+操纵
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


# 细化的添加结果
def append_to_csv2(file_path, json_file_path, is_success, path_length1, path_length2,NE1,NE2,is_success1,is_success2):
    # 创建新行的数据字典
    new_data = pd.DataFrame([{
        "json_file_path": json_file_path,
        "is_success": is_success,
        "path_length1": path_length1,
        "path_length2": path_length2,
        "NE1": NE1,
        "NE2": NE2,
        "is_success1": is_success1,
        "is_success2": is_success2
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


# 将任务对应的图片数据进行记录，创建对应的文件夹
def create_imgs_folder(json_file_path):
    json_file_dir = os.path.dirname(json_file_path)
    parent_folder = os.path.dirname(json_file_dir)
    folder_number = os.path.basename(json_file_dir)

    new_folder_name = f"{folder_number}_imgs"
    new_folder_path = os.path.join(parent_folder, new_folder_name)

    os.makedirs(new_folder_path, exist_ok=True)

    return new_folder_path


    

    
    
