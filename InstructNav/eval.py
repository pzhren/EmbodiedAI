import sys
import os
# Get the directory of the current file
current_dir = os.path.dirname(os.path.abspath(__file__))
# Navigate up to the EmbodiedAI directory
embodied_ai_dir = os.path.abspath(os.path.join(current_dir, '..', '..'))
# Add the EmbodiedAI directory to sys.path
sys.path.append(embodied_ai_dir)

os.environ['CUDA_VISIBLE_DEVICES'] = '0, 1'
import numpy as np
import argparse
import random
import csv
from tqdm import tqdm
from mapper import Instruct_Mapper
from objnav_agent import HM3D_Objnav_Agent
from simulator.core.config import EnvConfig
from simulator.core.env import BaseEnv
from simulator.core.dataset import DatasetLoader
from simulator.scenes import Interactive_Scene
from simulator.utils.scene_utils import extract_target_ids
from instructnav import InfiniteEnv
from lazyimport import lazyimport
lazyimport(globals(), """
	from omni.isaac.core.prims import XFormPrim
	from omni.isaac.core.robots import Robot
	from transformations import euler_from_quaternion,quaternion_from_euler
	"""
)

def write_metrics(metrics,path="benchmark3_hssd.csv"):
    with open(path, mode="w", newline="") as csv_file:
        fieldnames = metrics[0].keys()
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(metrics)

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--eval_episodes",type=int,default=500)
    parser.add_argument("--mapper_resolution",type=float,default=0.05)
    parser.add_argument("--path_resolution",type=float,default=0.2)
    parser.add_argument("--path_scale",type=int,default=5)
    return parser.parse_known_args()[0]


if __name__ == "__main__":
    args = get_args()
    loader = DatasetLoader(root_dir="resource/datasets/all_task",
				scene_path="your_scene_path",
				robot_path="resource/robots/stretch/stretch_pos.usd",
				headless = True)

    evaluation_metrics = []
    num_samples = 50
    indices = random.sample(range(len(loader)), num_samples)
    for i in tqdm(indices):
        cfg = loader[i]
        sim = BaseEnv(cfg)
        sim.reset()

        env = InfiniteEnv(sim)

        mapper = Instruct_Mapper(camera_intrinsic=env.camera_intrinsic,
                                    pcd_resolution=args.mapper_resolution,
                                    grid_resolution=args.path_resolution,
                                    grid_size=args.path_scale,
                                    floor_height=-1.3,
                                    ceiling_height=1.3)
        agent = HM3D_Objnav_Agent(env, mapper)
        agent.reset()
        agent.make_plan()

        while not env.done[0] and agent.episode_steps < 495:
            agent.step()

        agent.save_trajectory("./tmp/episode-%d/"%i)
        evaluation_metrics.append({'success':agent.metrics['success'],
                                'spl':agent.metrics['spl'],
                                'distance_to_goal':agent.metrics['distance_to_goal'],
                                'object_goal':agent.instruct_goal})
        write_metrics(evaluation_metrics)

        sim.close()