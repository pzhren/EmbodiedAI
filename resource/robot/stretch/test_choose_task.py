import os
os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES" # 设置环境变量以接受OMNI Kit的EULA（最终用户许可协议）
import shutil
import isaacsim
import math
import time
import pandas as pd
from omni.isaac.kit import SimulationApp

config = {
"width": "1280",
"height": "720",
"headless": False,
}
simulation_app = SimulationApp(config) # 配置仿真应用，指定窗口大小和是否无头模式


import omni.isaac.core.utils.bounds as bounds_utils
import carb
import numpy as np
import time
import math
import json
import numpy as np
from transformations import euler_from_quaternion
from omni.isaac.core import World
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import RigidPrim,RigidPrimView,GeometryPrim,XFormPrim
from omni.isaac.core.robots import Robot
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.sensor import Camera,RotatingLidarPhysX,IMUSensor
from omni.isaac.core.scenes import Scene as OmniBaseScene
import matplotlib.pyplot as plt
import omni.kit.viewport.utility as vu 
from omni.isaac.dynamic_control import _dynamic_control


from navigate_python import map_show,navigate,dstar_lite,discretize_map
from navigate_python.map_show import *
from navigate_python.navigate import *
from navigate_python.dstar_lite import *
from navigate_python.discretize_map import *
from BaseControl_position import BaseControl
from BaseGrasp_test import BaseGrasp
from semantic import SemanticMake

my_world = World(stage_units_in_meters=1.0)
reset_needed = False
assets_root_path = get_assets_root_path()
# 添加默认地面平面到场景中
my_world.scene.add_default_ground_plane()
# 获取并配置默认地面平面
plane = RigidPrim("/World/defaultGroundPlane")
plane.set_visibility(visible=False)
plane.set_local_pose(translation=np.array([0, 0, 1.1]))
plane.disable_rigid_body_physics() # 使地面不下坠


stretch_path = "/data1/lfwj/npc/SE3/stretch_fix/stretch_fix.usd"
stretch=prim_utils.create_prim(
prim_type="Xform",
prim_path = "/World/stretch",
usd_path=stretch_path,
translation=np.array([-2.5,9,0.0])
)

stretch = Robot(prim_path="/World/stretch")

# import demo scene
scene_root_path = "/data1/lfwj/hssd_scenes/final_selected_usd/"
# 读取 Excel 文件
xlsx_file_path = 'final_ID.xlsx' 
df = pd.read_excel(xlsx_file_path)

# 获取第一列的数值
ID_column = df.iloc[:, 0]  # iloc[:, 0]表示选择所有行的第一列
print(ID_column)
# 遍历不同的场景ID
for i in range(len(ID_column)):
    target_ids = str(ID_column[i])
    scene_ID = scene_root_path + target_ids + "/" + target_ids +".usd"
    # 删除已有的 prim
    
    if prim_utils.is_prim_path_valid("/World/Scene"):
        prim_utils.delete_prim("/World/Scene")

    construct_scene=prim_utils.create_prim(
        prim_type="Xform",
        prim_path = "/World/Scene",
        usd_path = scene_ID,
        scale=[1, 1, 1],
        translation=np.array([0, 0, 0]),
        orientation=[1,0,0,0]
    )
    
    scene = GeometryPrim("/World/Scene", collision=True)
    if prim_utils.is_prim_path_valid("/World/Light_3"):
        prim_utils.delete_prim("/World/Light_3")
    

    light_3 = prim_utils.create_prim(
        "/World/Light_3",
        "DomeLight",
        position=np.array([0.0, 0.0, 0.0]),
        attributes={
            "inputs:intensity": 1e3,
            "inputs:color": (1.0, 1.0, 1.0)
        }
    )

    robot_path = "/World/stretch/stretch"
    lift_path = "/World/stretch/link_mast"
    arm1_path = "/World/stretch/link_arm_l1"
    arm2_path = "/World/stretch/link_arm_l2"
    arm3_path = "/World/stretch/link_arm_l3"
    arm4_path = "/World/stretch/link_arm_l4"
    grasp_path = "/World/stretch/link_gripper_s3_body"



# 另起炉灶，不需要打开仿真APP
    # 获取场景的所有物品名字，prim_dict的键名为prim名字，值为对应的prim
    max_arm = 0.82 # 机械臂长度
    child = prim_utils.get_prim_children(construct_scene)
    child = prim_utils.get_prim_children(child[0])
    prim_list = []
    prim_dict = {}
    for prim in child:
        prim_list.extend(prim_utils.get_prim_children(prim))
    for prim in prim_list:
        prim_dict[prim.GetName()]=prim

    # 根据json文件读取物品ID
    json_files = []
    # 遍历base_path下的所有目录和子目录
    for dirpath, dirnames, filenames in os.walk('/data1/lfwj/npc/task/'+target_ids):
        # 对于每个文件，检查它是否以.json结尾
        for filename in filenames:
            if filename.endswith('.json'):
                # 如果是.json文件，添加到json_files列表中
                json_files.append(os.path.join(dirpath, filename))

    t = 0 # 任务个数
    for json_file in json_files:
        with open(json_file, 'r', encoding='utf-8') as file:
            json_data = json.load(file)

        # 提取目标物品的ID
        target_obj_ids = [item[0] for item in json_data['Target']]
        ID1,ID2 = target_obj_ids

        # 判断这个物品是否在场景文件中
        if ID1 not in prim_dict:
            if "_" + ID1[1:] not in prim_dict:
                print("_" + ID1[1:],"change_ID1 also not in")
                continue
            else:
                ID1 = "_" + ID1[1:]
                
        if ID2 not in prim_dict:
            if "_" + ID2[1:] not in prim_dict:
                print("_" + ID2[1:],"change_ID2 also not in")
                continue
            else:
                ID2 = "_" + ID2[1:]
        

        # 定义物品1属性字典
        obj_dict = {} 
        for attr in prim_dict[ID1].GetAttributes(): 
            obj_dict[attr.GetName()] = attr.Get()

        # 定义物品2的属性字典
        next_obj_dict = {} 
        for attr in prim_dict[ID2].GetAttributes(): 
            next_obj_dict[attr.GetName()] = attr.Get()


        # 物品ID,用于抓取
        obj_pos = obj_dict["xformOp:translate"]
        # 桌子ID,用于放置
        next_obj_pos = next_obj_dict["xformOp:translate"]

        # 第一个物体
        goal_obj = [obj_pos[0], obj_pos[1]]

        # 第二个物体
        next_goal_obj = [next_obj_pos[0], next_obj_pos[1]]

        # 读取地图
        img_root_path = "/data1/lfwj/hssd_scenes/final_selected_usd/"
        img_path = img_root_path + target_ids
        img_folder = os.listdir(img_path)
        # 过滤出所有以.png结尾的文件
        png_files = [file for file in img_folder if file.endswith('.png')]
        img_name = png_files[0]


        # 定义地图类
        hm = HeightMap(img_path+"/"+img_name)
        # hm.show_info()
        xy_range = hm.compute_range()
        hm.make_map()
        # hm.de_noising()
        hm_map = hm.get_map()

        # 定义导航类
        navigator = Navigator(area_range=xy_range, map=hm_map, scale_ratio=1)
        navigator.planner.compute_cost_map()
        
    # 获取实际导航到达位置
        print("开始寻找点")
        tmp_pos = navigator.planner.real2map(goal_obj)
        print("找到点")
        final_pos = navigator.planner.map2real(tmp_pos)

        dist = math.sqrt((goal_obj[0] - final_pos[0])**2 + (goal_obj[1] - final_pos[1])**2)
        if dist > max_arm:
            print("ID1_dist:",dist)
            continue

        print("开始寻找点")
        tmp_pos = navigator.planner.real2map(next_goal_obj)
        print("找到点")
        final_pos = navigator.planner.map2real(tmp_pos)

        dist = math.sqrt((next_goal_obj[0] - final_pos[0])**2 + (next_goal_obj[1] - final_pos[1])**2)
        if dist > max_arm:
            print("ID2_dist:",dist)
            continue
        else:
            print(json_file)
            t += 1
            target_path = "/data1/lfwj/npc/selected_task2/" + target_ids + "/" + str(t) +"/"
            # 创建目标文件夹
            os.makedirs(os.path.dirname(target_path), exist_ok=True)
            target_path = target_path + "config.json"
            shutil.copy2(json_file, target_path)
            print(f"文件 {json_file} 已成功移动到 {target_path}")
    print("stop")
print("final_stop")

