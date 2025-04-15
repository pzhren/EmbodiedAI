import os
from simulator.core.config import EnvConfig
from simulator.core.env import BaseEnv
from simulator.core.dataset import DatasetLoader
from simulator.scenes import Interactive_Scene
from simulator.utils.scene_utils import extract_target_ids
from lazyimport import lazyimport
lazyimport(globals(), """
    from omni.isaac.core.prims import XFormPrim
    from omni.isaac.core.robots import Robot
    from transformations import euler_from_quaternion,quaternion_from_euler
  """
)
print(os.getcwd())

# config_file= "tests/test_configs/test.yaml"
# cfg = EnvConfig(config_file)
# print(cfg.config)
loader = DatasetLoader(root_dir="/data1/linmin/EmbodiedAI/resource/datasets/all_task",
scene_path="/data1/linmin/NPC/hssd_test_scene",
robot_path="/data1/linmin/EmbodiedAI/tests/stretch/model/stretch.usd",
headless = True)
print("total len:",len(loader))
cfg = loader[422]
cfg.task.robots[0].position= [-10, 6, 0]
print(cfg)
env = BaseEnv(cfg)
env.reset()

while env.is_running:
        env.sim._warm_up()
        # print(cfg.config.task.task_path)
        target_ids = extract_target_ids(cfg.task.task_path)
        objs_xformprim = env.sim.find_object_by_id(env.scenes[0], target_ids)
        
        # 获取当前的机器人的位置
        current_pos = env.robots[0][0].get_world_pose()
        
        current_pos = current_pos[:2]
        
        # around_objects = env.sim.find_object_around(env.scenes[0], current_pos)
        # print("around_objects", around_objects)

        goal_pos1, _ = objs_xformprim[0].get_world_pose()
        
        print("goal_pos1", goal_pos1)
        # 这里很奇怪，要全部加负号才正常
        goal_pose1 = [-goal_pos1[0], -goal_pos1[1]]
        # distance, action, action_value = env.task[0].get_distance(goal_pose1)
        action = "w"
        action_value = 0.02
        # print("distance, action", distance, action, action_value)
        obs, info, reward, done = env.step([[action, action_value]])
        # 把obs的观测数据按照图片保存下来
        rgb1 = obs[0]["robot0_front_camera"]["rgb"]
        rgb2 = obs[0]["robot0_left_camera"]["rgb"]
        rgb3 = obs[0]["robot0_right_camera"]["rgb"]
        
        # depth1 = obs[0][0][0]["depth"]
        # depth2 = obs[0][0][1]["depth"]
        # depth3 = obs[0][0][2]["depth"]
        
        # 将这些图片保存下来，利用Pillow库
        from PIL import Image
        Image.fromarray(rgb1).save(f"/data1/linmin/EmbodiedAI/tests/obs/rgb1.png")
        Image.fromarray(rgb2).save(f"/data1/linmin/EmbodiedAI/tests/obs/rgb2.png")
        Image.fromarray(rgb3).save(f"/data1/linmin/EmbodiedAI/tests/obs/rgb3.png")
        i+=1