from navigate_python.map_show import *
from navigate_python.navigate import *
from navigate_python.dstar_lite import *
from navigate_python.discretize_map import *



def main(json_file):
    import os
    os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES" # 设置环境变量以接受OMNI Kit的EULA（最终用户许可协议）
    import isaacsim
    import time
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
    import math
    import json
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
    from BaseControl_position import BaseControl
    from BaseGrasp_test import BaseGrasp
    from transformations import euler_from_quaternion,quaternion_from_euler
    from some_function import is_success, is_success2,init_robot_pos,calculate_path_length, create_imgs_folder
 
    # 导入场景
    scene_root_path = "/data1/lfwj/hssd_scenes/final_selected_usd/"
    
    # 创建记录过程图片的文件夹
    process_img_folder = create_imgs_folder(json_file)

    # 提取目标场景的ID1
    with open(json_file, 'r', encoding='utf-8') as file:
        json_data = json.load(file)
    
    scene_id = json_data['Scene']
    scene_path = scene_root_path + scene_id + "/" + scene_id +".usd"
    
    # 创建世界
    my_world = World(stage_units_in_meters=1.0)
    reset_needed = False
    assets_root_path = get_assets_root_path()
    # 添加默认地面平面到场景中
    my_world.scene.add_default_ground_plane()
    # 获取并配置默认地面平面
    plane_path = "/World/defaultGroundPlane"
    plane = RigidPrim(plane_path)
    plane.set_visibility(visible=False)
    plane.set_local_pose(translation=np.array([0, 0, 1.1]))
    plane.disable_rigid_body_physics() # 使地面不下坠


    # 获取初始时机器人的位置，通过读取地图获得
    img_path = "/data1/lfwj/hssd_scenes/final_selected_usd/" + str(scene_id)
    img_folder = os.listdir(img_path)
    # 过滤出所有以.png结尾的文件
    png_files = [file for file in img_folder if file.endswith('.png')]
    img_name = png_files[0]
    stretch_pos = init_robot_pos(img_path=img_path+"/"+img_name)
    print("机器人位置确定")

    stretch_path = "/data1/lfwj/npc/stretch_linmin/stretch_pos.usd"
    stretch=prim_utils.create_prim(
    prim_type="Xform",
    prim_path = "/World/stretch",
    usd_path=stretch_path,
    translation = np.array([stretch_pos[0],stretch_pos[1],0.0])
    )

    stretch = Robot(prim_path="/World/stretch")

    construct_scene=prim_utils.create_prim(
        prim_type="Xform",
        prim_path = "/World/Scene",
        usd_path = scene_path,
        scale=[1, 1, 1],
        translation=np.array([0, 0, 0]),
        orientation=[1,0,0,0]
    )
    scene = GeometryPrim("/World/Scene", collision=True)


# 整体打灯
    light_3 = prim_utils.create_prim(
        "/World/Light_3",
        "DomeLight",
        position=np.array([0.0, 0.0, 0.0]),
        attributes={
            # "inputs:radius": 0.5,
            "inputs:intensity": 1e3,
            "inputs:color": (1.0, 1.0, 1.0)
        }
    )
    camera = Camera("/World/stretch/camera_color_frame/rgb_camera",
                        position = np.array([stretch_pos[0],stretch_pos[1], 1.0]),
                        resolution=(1280,640),
                        frequency=30,
                        orientation = np.array([1, 0, 0, 0]))

    imu_sensor = my_world.scene.add(
        IMUSensor(
            prim_path="/World/stretch/base_imu/imu_sensor",
            name="imu",
            frequency=60,
            translation=np.array([0, 0, 0]),
        )
    )
    camera.initialize()
    camera.add_motion_vectors_to_frame() 
    camera.add_distance_to_image_plane_to_frame()
    camera.set_focal_length(31.17691)
    camera.set_horizontal_aperture(36.0)


    robot_path = "/World/stretch"
    lift_path = "/World/stretch/link_mast"
    arm1_path = "/World/stretch/link_arm_l1"
    arm2_path = "/World/stretch/link_arm_l2"
    arm3_path = "/World/stretch/link_arm_l3"
    arm4_path = "/World/stretch/link_arm_l4"
    grasp_path = "/World/stretch/link_gripper_s3_body"
    
    img_index = 0
    while simulation_app.is_running():
        my_world.step(render=True)
        if my_world.is_stopped() and not reset_needed:
            reset_needed = True
        my_world.play()
        if my_world.is_playing():
            if reset_needed:
                my_world.reset()
                reset_needed = False

            # 提取先验信息
                # 获取场景的所有物品名字，prim_dict的键名为prim名字，值为对应的prim
                child = prim_utils.get_prim_children(construct_scene)
                child = prim_utils.get_prim_children(child[0])
                prim_list = []
                prim_dict = {}
                for prim in child:
                    prim_list.extend(prim_utils.get_prim_children(prim))
                for prim in prim_list:
                    prim_dict[prim.GetName()]=prim
                
                # 提取目标物品的ID
                target_ids = [item[0] for item in json_data['Target']]
                ID1,ID2 = target_ids

                # 判断两个物品的ID是否在场景字典中，否则使用_开头版本
                if ID1 not in prim_dict:
                    ID1 = "_" + ID1[1:]
                            
                if ID2 not in prim_dict:
                    ID2 = "_" + ID2[1:]

                # 物品ID,用于抓取
                obj_path = "/World/Scene/floorplan/furniture/"  + ID1

                # 桌子ID,用于放置
                obj_next_path = "/World/Scene/floorplan/furniture/"  + ID2
                
                # 定义物品1属性字典
                obj_dict = {} 
                for attr in prim_dict[ID1].GetAttributes(): 
                    obj_dict[attr.GetName()] = attr.Get()

                # 定义物品2的属性字典
                next_obj_dict = {} 
                for attr in prim_dict[ID2].GetAttributes(): 
                    next_obj_dict[attr.GetName()] = attr.Get()
        
            # 定义物品的prim
                r,p,y = obj_dict["xformOp:rotateXYZ"]
                quaternion1 = quaternion_from_euler(r/180 * math.pi,p/180 * math.pi,y/180 * math.pi)
                obj_xformprim = XFormPrim(obj_path,translation=obj_dict["xformOp:translate"],orientation=quaternion1) 

                r,p,y = next_obj_dict["xformOp:rotateXYZ"]
                quaternion2 = quaternion_from_euler(r/180 * math.pi,p/180 * math.pi,y/180 * math.pi)
                obj_next_xformprim = XFormPrim(obj_next_path,translation=next_obj_dict["xformOp:translate"],orientation=quaternion2) 
            
            # 得到物品的实际位置
                goal_obj_3, orientation = obj_xformprim.get_world_pose()
                goal_obj = [goal_obj_3[0], goal_obj_3[1]]

            # 定义地图类
                img_root_path = "/data1/lfwj/hssd_scenes/final_selected_usd/"
                img_path = img_root_path + scene_id
                img_folder = os.listdir(img_path)
                # 过滤出所有以.png结尾的文件
                png_files = [file for file in img_folder if file.endswith('.png')]
                img_name = png_files[0]
                hm = HeightMap(img_path+"/"+img_name)
                xy_range = hm.compute_range()
                hm.make_map()
                hm_map = hm.get_map()

            # 定义导航类
                navigator = Navigator(area_range=xy_range, map=hm_map, scale_ratio=1)
                navigator.planner.compute_cost_map()

                stretch_1 = XFormPrim("/World/stretch")
                position = stretch_1.get_default_state().position
                position_start = [position[0],position[1]]

            # 定义移动类
                base_control = BaseControl(robot=robot_path,obj_prim=obj_xformprim,next_obj_prim=obj_next_xformprim)
            # 定义抓取类
                grasp_test = BaseGrasp(arm1=arm1_path,arm2=arm2_path,arm3=arm3_path,arm4=arm4_path,lift=lift_path,grasp=grasp_path,obj_prim=obj_xformprim,robot=robot_path)
                
            # 开始路径规划
                path, map_path = navigator.navigate(goal_obj, position_start)

            # # 保存路径规划图像
            #     # 目标点地图坐标
            #     map_goal_pos = navigator.planner.real2map(goal_obj)
            #     show_map_(navigator.planner.cost_map, map_path, map_goal_pos,save_path=process_img_folder + "/"+ "path1.png")


            # 计算路径长度
                min_path_length1 = calculate_path_length(path=path)
            # 计算第一段导航的偏差
                NE1 = math.sqrt((goal_obj[0] - path[-1][0])**2 + (goal_obj[1] - path[-1][1])**2)


            # 移动到指定位置进行抓取
                for i in range(len(path)):
                    tmp = [path[i][0],path[i][1],0]
                    base_control.move_by_target(tmp, my_world,orientation=False)# False表示移动过程中不需要转换位姿态

                    # 获取相机照片
                    img_index += 1
                    if camera.get_rgb().any():
                        plt.imsave(process_img_folder + "/"+f"frame_{img_index:03d}.png",camera.get_rgb())

                    my_world.step(render=True)
            
            # 判断物品和机器人之间隔着墙壁
                cur_position, cur_orientation = stretch_1.get_world_pose()
                if is_success(img_path=img_path+"/"+img_name,obj_world_pos=goal_obj,robot_world_pos=[cur_position[0],cur_position[1]]):
                    sub_success1 = 1
                else:
                    sub_success1 = 0


            # 抓取操作
                base_control.turn4grasp(world=my_world) # 转到合适位置进行抓取

                # 获取相机照片
                img_index += 1
                if camera.get_rgb().any():
                    plt.imsave("/data1/lfwj/robot_image/"+f"frame_{img_index:03d}.png",camera.get_rgb())

                cur_position, cur_orientation = stretch_1.get_world_pose()
                arg4grasp = grasp_test.transform_to_base(pos = cur_position, orient = cur_orientation, target = goal_obj_3) # 获得机械臂参数
                grasp_test.grasp_by_target(target=arg4grasp, world=my_world) # 伸到指定位置抓取


            # 抓取之后开始移动
                position,_ = stretch_1.get_world_pose()
                position_start = [position[0],position[1]]

                goal_next_obj_3, orientation_ = obj_next_xformprim.get_world_pose()
                goal_next_obj = [goal_next_obj_3[0], goal_next_obj_3[1]]

            # 开始路径规划
                print("新路径规划")
                new_obj_goal = goal_next_obj
                path, map_path = navigator.navigate(new_obj_goal, position_start)
            
            # # 保存路径规划图像
            #     # 目标点地图坐标
            #     map_goal_pos = navigator.planner.real2map(goal_next_obj)
            #     show_map_(navigator.planner.cost_map, map_path, map_goal_pos,save_path=process_img_folder + "/"+ "path2.png")

            # 计算路径长度
                min_path_length2 = calculate_path_length(path=path)
            # 计算第二段导航的偏差
                NE2 = math.sqrt((new_obj_goal[0] - path[-1][0])**2 + (new_obj_goal[1] - path[-1][1])**2)

            # 移动到指定位置
                for i in range(len(path)):
                    tmp = [path[i][0],path[i][1],0]
                    base_control.move_by_target(tmp, my_world,is_grasp=grasp_test.is_grasping,orientation=False)# 不需要转换位姿态

                    # 获取相机照片
                    img_index += 1
                    if camera.get_rgb().any():
                        plt.imsave(process_img_folder + "/"+f"frame_{img_index:03d}.png",camera.get_rgb())

                    my_world.step(render=True)
                my_world.step(render=True)


            # 调整释放物体位姿
                base_control.turn4release(world=my_world,is_grasp=grasp_test.is_grasping,obj_prim=obj_next_xformprim)

                # 获取相机照片
                img_index += 1
                if camera.get_rgb().any():
                    plt.imsave(process_img_folder + "/"+f"frame_{img_index:03d}.png",camera.get_rgb())

                my_world.step(render=True)

            # 释放物品
                cache = bounds_utils.create_bbox_cache() # 获取bounding box位置坐标
                bounding_box = bounds_utils.compute_aabb(cache, prim_path=obj_next_path)
                bounding_minx,bounding_box_miny,min_z,bounding_maxx,bounding_maxy,max_z = bounding_box

                bounding_range = [bounding_minx,bounding_box_miny, bounding_maxx, bounding_maxy]

                # 桌子边缘范围

                cur_position, cur_ori = stretch_1.get_world_pose()
                cur_x,cur_y = cur_position[0],cur_position[1]
                arg4release = grasp_test.transform_to_base(pos = cur_position, orient = cur_ori, target = goal_next_obj_3) # 获得机械臂参数
                grasp_test.release_by_target(target=[arg4release[0],max_z],world=my_world)

                # 获取相机照片
                img_index += 1
                if camera.get_rgb().any():
                    plt.imsave(process_img_folder + "/"+f"frame_{img_index:03d}.png",camera.get_rgb())

                obj_cur_pos,_ = obj_xformprim.get_world_pose()
                obj_cur_pos = [obj_cur_pos[0],obj_cur_pos[1]]
                robot_cur_pose = [cur_x,cur_y]

            # 判断桌子和机器人之间是否隔着墙壁
                if is_success(img_path=img_path+"/"+img_name,obj_world_pos=goal_next_obj,robot_world_pos=[cur_position[0],cur_position[1]]):
                    sub_success2 = 1
                else:
                    sub_success2 = 0

                my_world.step(render=True)
                if is_success2(img_path=img_path+"/"+img_name,obj_world_pos=obj_cur_pos,robot_world_pos=robot_cur_pose,next_obj_range=bounding_range):
                    print("success")  
                    # return True, min_path_length1 + min_path_length2, (NE1 + NE2)/2
                    return True, min_path_length1, min_path_length2, NE1, NE2, sub_success1, sub_success2
                else:
                    print("Fail")
                    # return False, min_path_length1 + min_path_length2, (NE1 + NE2)/2
                    return False, min_path_length1, min_path_length2, NE1, NE2, sub_success1, sub_success2
    simulation_app.close()



json_path ="/data1/lfwj/npc/selected_task2/102344049/2/config.json"
# state, total_path_len, average_NE=  main(json_path)
# print(state, total_path_len, average_NE)
# if state:
#     print("state:success")  
# else:
#     print("state:Fail")

# # 将结果填入表格
# from some_function import append_to_csv
# csv_file = "result.csv"
# append_to_csv(file_path=csv_file,json_file_path=json_path,is_success=str(state),path_length=total_path_len,NE=average_NE)

state, path_len1, path_len2,  NE1, NE2, sub_success1, sub_success2=  main(json_path)
print(state, path_len1, path_len2,NE1,NE2,sub_success1, sub_success2)
if state:
    print("state:success")  
else:
    print("state:Fail")

# 将结果填入表格
from some_function import append_to_csv2
csv_file = "result2.csv"
append_to_csv2(file_path=csv_file,json_file_path=json_path,is_success=str(state),path_length1=path_len1,path_length2=path_len2,NE1=NE1,NE2=NE2,is_success1=sub_success1,is_success2=sub_success2)

