import os
from simulator.core.config import EnvConfig
from simulator.core.env import BaseEnv
from simulator.core.dataset import DatasetLoader
from simulator.scenes import Interactive_Scene
from simulator.utils.scene_utils import extract_target_ids
from simulator.agents.random import RandomAgent
import json

loader = DatasetLoader(root_dir="/data1/linmin/EmbodiedAI/resource/datasets/all_task",
scene_path="/data1/linmin/NPC/hssd_test_scene",
robot_path="/data1/linmin/EmbodiedAI/tests/stretch/model/stretch.usd",
headless = True)
cfg = loader[0]
task_path = cfg.task.task_path
env = BaseEnv(cfg)
print("total len:",len(loader))
for i in range(len(loader)):
    cfg = loader[i]
    task_path = cfg.task.task_path
    print(task_path)
    env.reinit(loader[i])
    obs = env.reset()
    agent = RandomAgent(config=None)
    agent.reset()
    while env.is_running:
        with open(task_path, "r") as f:
           orin_task = json.load(f)
        orin_task["Optimal Length"] = env.task[0].optimal_length
        with open(task_path, "w") as f:
           json.dump(orin_task, f)
        env.close()