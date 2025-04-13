import os
from simulator.core.config import EnvConfig
from simulator.core.env import BaseEnv
from simulator.core.dataset import DatasetLoader
from simulator.scenes import Interactive_Scene
from simulator.utils.scene_utils import extract_target_ids
from simulator.agents.random import RandomAgent


loader = DatasetLoader(root_dir="/data1/linmin/EmbodiedAI/resource/datasets/all_task",
scene_path="/data1/linmin/NPC/hssd_test_scene",
robot_path="/data1/linmin/EmbodiedAI/tests/stretch/model/stretch.usd",
headless = True)
print("total len:",len(loader))
cfg = loader[422]
env = BaseEnv(cfg)
obs = env.reset()
agent = RandomAgent(config=None)
agent.reset()
while env.is_running:
    action = agent.act(obs)
    obs, reward, done, info = env.step(action)