import os
import math
import json
from PIL import Image
import argparse
import csv
import cv2
import time
import quaternion
import numpy as np
from tqdm import tqdm
from constants import *
from mapping_utils.transform import habitat_camera_intrinsic
from mapper import Instruct_Mapper
from objnav_agent import HM3D_Objnav_Agent
# from utils.dataset import count_gt_path

# os.environ['CUDA_VISIBLE_DEVICES'] = '1,2'
# os.environ["MAGNUM_LOG"] = "quiet"
# os.environ["HABITAT_SIM_LOG"] = "quiet"


def trans_pos(position):
    return np.array([position[0], position[2], position[1]])

def trans_yaw(yaw):
    return quaternion.quaternion(yaw[0], yaw[1], yaw[2], yaw[3])


class InfiniteEnv:
    def __init__(self, sim):
        self.sim = sim

        self.ins, self.targets, self.scene_map = self.read_config()
        print("task:", self.ins)

        self.target_states = self.sim.configs.task.goal_points

        self.pos = trans_pos(self.sim.robots[0][0].get_world_pose())
        self.yaw = trans_yaw(self.sim.robots[0][0].get_world_orientation())
        self.camera_pos = trans_pos(self.sim.robots[0][0].sensors[0].get_world_pose())
        self.camera_yaw = trans_yaw(self.sim.robots[0][0].get_world_orientation())

        self.observation = [None]
        self.info = [None]
        self.reward = [None]
        self.done = [False]

        self.camera_intrinsic = self.sim.robots[0][0].sensors[0].camera.get_intrinsics_matrix()
        # self.counter = 0
        self.actor([0, 0])
        self.actor([0, 0])

    def actor(self, action):
        self.sim.step([action])
        time.sleep(abs(action[1]))
        self.observation, self.info, self.reward, self.done = self.sim.step([[0,0]])

        _, _, _, yaw = self.sim.task[0].trans_pos()
        yaw = yaw * 180 / math.pi
        print(f"agent pos: {self.sim.robots[0][0].get_world_pose()}, agent yaw: {yaw}")
        print(f"info: {self.info[0]}")
        # rgb = self.observation[0]["robot0_front_camera"]["rgb"] 
        # cv2.imwrite(f"{self.counter}_rgb_{yaw}.png", rgb)
        # self.counter = self.counter + 1

        self.pos = trans_pos(self.sim.robots[0][0].get_world_pose())
        self.yaw = trans_yaw(self.sim.robots[0][0].get_world_orientation())
        self.camera_pos = trans_pos(self.sim.robots[0][0].sensors[0].get_world_pose())
        self.camera_yaw = trans_yaw(self.sim.robots[0][0].get_world_orientation())

        return self.observation[0], self.info[0], self.reward[0], self.done[0]
        
    def read_config(self):
        config_file = self.sim.configs.task.task_path
        with open(config_file, 'r') as f:
            config = json.load(f)
        target = config['Target']
        return config['Task instruction'], [t[1] for t in target], [t[3] for t in target]


    def get_topdown_config(self):
        scene = '/'.join(self.sim.configs.scene.scene_file.split('/')[:-1])
        topdown_map = None
        for file in os.listdir(scene):
            if file.endswith('.png'):
                topdown_map = np.array(Image.open(os.path.join(scene, file)).convert("RGB"))
        
        coord, _, _, angle = self.sim.task[0].trans_pos()
        map_coord = [self.sim.task[0].navigator.planner.real2map(coord[:2])]
        map_angle = [angle*180 / math.pi]

        mask = None
        
        topdown_config = {
            "map": topdown_map,
            "fog_of_war_mask": mask,
            "agent_map_coord": map_coord,
            "agent_angle": map_angle,
        }

        return topdown_config
        
    def planner(self, pos):
        total_distance, action_list = self.sim.task[0].get_distance_list(pos)
        return action_list

    def get_metrics(self):
        return {
            'success': self.done[0],
            'spl': self.info[0]['SPL'][0][0],
            'soft_spl': self.info[0]['SPL'][0][1],
            'distance_to_goal': self.info[0]['NE'][0][0] if self.info[0]['NE'][0][1] == 0 else sum(self.info[0]['NE'][0])/2 ,
        }

            
