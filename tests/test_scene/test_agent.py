import os
from simulator.core.config import EnvConfig, AgentConfig
from simulator.core.env import BaseEnv
from simulator.core.dataset import DatasetLoader
from simulator.scenes import Interactive_Scene
from simulator.utils.scene_utils import extract_target_ids
from simulator.agents import BCAgent, ReferencePathAgent

loader = DatasetLoader(root_dir="/data1/linmin/EmbodiedAI/resource/datasets/all_task",
scene_path="/data1/linmin/NPC/hssd_test_scene",
robot_path="/data1/linmin/EmbodiedAI/resource/robots/stretch/stretch_pos.usd",
headless = True)
print("total len:",len(loader))
cfg = loader[422]
env = BaseEnv(cfg)

# agent = ReferencePathAgent(cfg)
# # Create config with checkpoint path
config = AgentConfig(name="bc_agent", type="bc_agent", checkpoint_path="")
config.checkpoint_path = "/data1/linmin/imitation_learning_project/no_gen_scene_checkpoints/checkpoint_epoch_95.pt"  # Update this path

# Create agent
agent = BCAgent(config)



obs = env.reset()
agent.reset()
i = 0
done = False
while env.is_running and not done:
    i+=1
    target_ids = extract_target_ids(cfg.task.task_path)
    objs_xformprim = env.sim.find_object_by_id(env.scenes[0], target_ids)
    goal_pos1, _ = objs_xformprim[0].get_world_pose()
        
    print("goal_pos1", goal_pos1)
    action = agent.act(obs[0])
    print(action)
    if i>80:
        print(i)
    print(obs[0]["position"])
    obs, reward, done, info = env.step([action])
    rgb1 = obs[0]["robot0_front_camera"]["rgb"]
    rgb2 = obs[0]["robot0_left_camera"]["rgb"]
    rgb3 = obs[0]["robot0_right_camera"]["rgb"]
    print(info)
    done = done[0]
    from PIL import Image
    Image.fromarray(rgb1).save(f"/data1/linmin/EmbodiedAI/tests/obs/rgb1_{i}.png")
    Image.fromarray(rgb2).save(f"/data1/linmin/EmbodiedAI/tests/obs/rgb2_{i}.png")
    Image.fromarray(rgb3).save(f"/data1/linmin/EmbodiedAI/tests/obs/rgb3_{i}.png")