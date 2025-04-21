## Overview

 Our simulator relies on the underlying API of Isaac Sim to operate, enabling high-quality rendering and realistic physical simulation.
 
___

## Docker

We provide a Docker image with our simulator interface already installed. You can pull it using the following command after installing [Docker](https://docs.docker.com/engine/install/).

```python
docker pull twilighted/embodiedai
```

## Base Modules and Classes
###  **1. Environment**
#### **EnvConfig**

**Description:** A class for managing the loading of environment configurations. 

**Init Parameters:**
- `config_path`(str): Path to the JSON file for environment settings

**Method**:
- `load_config()`: Load the config form the JSON file

**Attributes:**
- `config`(pydantic.BaseModel): Relevant parameters of the environment, including simulator settings, scenarios, robots, controllers, sensors, etc.


#### **BaseEnv**

**Description:** A class that manages the simulation environment, directly interacting with the underlying API of the simulator.

**Init Parameters:**
- `config`(pydantic.BaseModel): Relevant parameters of the environment, including simulator settings, scenarios, robots, controllers, sensors, etc.

**Method**:
- `step()`: All robots in the environment perform actions 
        **parameters**: Action(np.ndarray)
        **returns**: Observation(dict)
- `find_object_around()`: Get all objects near a certain point location
        **parameters**: Robot Position(np.ndarray)
        **returns**: Object List(List)
- `close()`: Close the environment

**Attributes:**
	- `config`(pydantic.BaseModel): Relevant parameters of the environment, including simulator settings, scenarios, robots, controllers, sensors, etc.
	- `sim` (Simulator): Class for managing the simulator.
	- `robots`(List): List of all robots of the environment. 
	- `scenes`(List): List of all scenes of the environment.
	- `task_objects`(List): List of objects involved in all tasks

#### **DatasetLoader**

**Description:** A data loading class that merges task and scene datasets to generate a list of environment configuration dictionaries.

**Init Parameters:**
- `root_dir`(str): Path specifying the location of the task dataset.
- `scene_path`(str): Path specifying the location of the scene dataset.
- `shuffle`(bool):  Whether to randomly shuffle the dataset when loading.
- `device_id`(int): The GPU ID used for loading scenes. Defaults to 0.
- `headless`(bool):  Whether to enable display mode. Default is True.

**Example**
```python
from simulator.core.dataset import DatasetLoader
loader = DatasetLoader(root_dir="resource/datasets/all_task",
				scene_path="/data1/linmin/NPC/hssd_test_scene",
				robot_path="tests/stretch/model/stretch.usd",
				headless = False)
				
print("total len:", len(loader))
cfg = loader[422]
print("config:", cfg)
```

#### **Agent**

**Description:**  An agent class used to process observations and generate actions.

**Init Parameters:**
- `config`: Custom agent configuration

**Example**
```python
from simulator.agents.random import RandomAgent
agent = RandomAgent()
agent.reset()
action = agent.act(Observation=None)
observation, info, reward, done = env.step(action)
```
You can define your own agent to process observations and generate actions for evaluation.

#### **Base Usage**

An example to demonstrate how to move the robot and retrieve observations and other information.


**Example**
```python
from simulator.core.config import EnvConfig
from simulator.core.env import BaseEnv
from simulator.core.dataset import DatasetLoader

loader = DatasetLoader(root_dir="resource/datasets/all_task",
				scene_path="/data1/linmin/NPC/hssd_test_scene",
				robot_path="tests/stretch/model/stretch.usd",
				headless = False)
				
env = BaseEnv(cfg)

env.reset()

while env.is_running:
    for _ in range(10):_
		action = "w" # move forward 0.02m
        action_value = 0.02 
        obs, info, reward, done = env.step([[action, action_value]])
        # front camera
		rgb1 = obs[0]["robot0_front_camera"]["rgb"] 
		# left camera
		rgb2 = obs[0]["robot0_left_camera"]["rgb"]
		# right camera 
		rgb3 = obs[0]["robot0_right_camera"]["rgb"]
	env.close()
	    
```


## Contact & Support

For support or additional questions, please contact linm57@mail2.sysu.edu.cn or visit our [documentation portal](https://github.com/pzhren/InfiniteWorld).