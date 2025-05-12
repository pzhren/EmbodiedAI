# EmbodiedAI Simulator API 文档

本项目基于 NVIDIA Isaac Sim，提供高质量渲染与真实物理仿真能力。以下为主要接口、功能及参数说明，便于开发者集成与二次开发。

---

## 目录
- [环境配置与加载](#环境配置与加载)
- [环境管理 BaseEnv](#环境管理-baseenv)
- [数据集加载 DatasetLoader](#数据集加载-datasetloader)
- [智能体 Agent](#智能体-agent)
- [典型用例](#典型用例)
- [控制器 Controllers](#控制器-controllers)
- [传感器 Sensors](#传感器-sensors)
- [任务 Tasks](#任务-tasks)
- [机器人 Robots](#机器人-robots)

---

## 环境配置与加载

### EnvConfig
- **功能**：加载环境配置（YAML/JSON），并生成标准化配置对象。
- **初始化参数**：
  - `config_path` (str): 配置文件路径。
- **主要方法**：
  - `load_config()`: 加载配置文件。
- **主要属性**：
  - `config` (pydantic.BaseModel): 包含仿真器、场景、机器人、控制器、传感器等参数。

---

## 环境管理 BaseEnv

### BaseEnv
- **功能**：管理仿真环境，直接与底层仿真API交互。
- **初始化参数**：
  - `config` (pydantic.BaseModel): 环境参数。
- **主要方法**：
  - `step(action)`: 所有机器人执行动作。
    - **参数**：
      - `action` (np.ndarray 或 List): 机器人动作。
    - **返回**：
      - `obs_list` (List[dict]): 观测结果。
      - `info_list` (List[dict]): 额外信息。
      - `reward_list` (List[float]): 奖励。
      - `done_list` (List[bool]): 是否结束。
  - `find_object_around(position, scene_id=0)`: 获取指定位置附近的物体。
    - **参数**：
      - `position` (np.ndarray): 位置。
      - `scene_id` (int): 场景编号。
    - **返回**：
      - `object_list` (List): 物体列表。
  - `reset(scene_id=0)`: 重置环境。
  - `close()`: 关闭环境。
- **主要属性**：
  - `robots` (List): 环境中的机器人列表。
  - `scenes` (List): 场景列表。
  - `task_objects` (List): 任务相关物体。

---

## 数据集加载 DatasetLoader

### DatasetLoader
- **功能**：合并任务与场景数据集，生成环境配置字典列表。
- **初始化参数**：
  - `root_dir` (str): 任务数据集路径。
  - `scene_path` (str): 场景数据集路径。
  - `robot_path` (str): 机器人模型路径。
  - `shuffle` (bool): 是否打乱数据集。
  - `device_id` (int): GPU编号。
  - `headless` (bool): 是否无头模式。
- **主要方法**：
  - `__getitem__(index)`: 获取第 index 个环境配置。
  - `__len__()`: 数据集长度。
  - `reset_cache()`: 清空缓存。

---

## 智能体 Agent

### BaseAgent
- **功能**：智能体基类，定义观测-动作接口。
- **主要方法**：
  - `reset()`: 重置智能体。
  - `act(observation)`: 根据观测生成动作。
    - **参数**：
      - `observation` (dict): 观测。
    - **返回**：
      - `action` (Any): 动作。

### RandomAgent (示例)
- **功能**：随机动作智能体。
- **主要方法**：
  - `act(observation)`: 随机采样动作。
  - `reset()`: 重置。

---

## 典型用例

```python
from simulator.core.config import EnvConfig
from simulator.core.env import BaseEnv
from simulator.core.dataset import DatasetLoader
from simulator.agents.random import RandomAgent

# 加载数据集
loader = DatasetLoader(root_dir="resource/datasets/all_task",
                      scene_path="/data1/linmin/NPC/hssd_test_scene",
                      robot_path="tests/stretch/model/stretch.usd",
                      headless=False)

cfg = loader[0]

# 创建环境
env = BaseEnv(cfg)

# 创建智能体
agent = RandomAgent(config=None)

obs = env.reset()
while env.is_running:
    action = agent.act(obs)
    obs, info, reward, done = env.step([action])
    if done[0]:
        break
env.close()
```

---

## 联系与支持
如有疑问请联系 linm57@mail2.sysu.edu.cn 或访问 [文档主页](https://github.com/pzhren/InfiniteWorld)。

---

## 控制器 Controllers

### BaseController
- **功能**：所有控制器的抽象基类，定义动作生成的标准接口。
- **典型属性**：
  - `config` (ControllerConfig): 控制器配置。
  - `type` (str): 控制器类型。
  - `input_limit` (Any): 输入限制。
  - `output_limit` (Any): 输出限制。
- **主要方法**：
  - `get_action(command, robot=None) -> np.ndarray`
    - **参数**：
      - `command` (list/str): 控制指令（如移动、旋转等）。
      - `robot` (可选): 机器人实例。
    - **返回**：
      - `np.ndarray` 或 tuple: 目标动作。
    - **说明**：子类需实现，生成具体动作。
  - `action_space` (property): 返回控制器动作空间（如gym.spaces.Box/Tuple等）。
  - `action_dim` (property): 返回动作维度（int）。

### PositionController
- **功能**：实现基于位移和角度的移动控制。
- **典型属性**：
  - `_max_forward_m` (float): 最大前进距离。
  - `_max_angle_yaw` (float): 最大旋转角度。
- **主要方法**：
  - `get_action(command, robot=None) -> (np.ndarray, np.ndarray)`
    - **参数**：
      - `command` (list): [动作类型, 参数值]，如['w', 0.02]表示前进0.02米。
      - `robot`: 机器人实例。
    - **返回**：
      - `target_pos` (np.ndarray): 目标位置。
      - `euler` (np.ndarray): 目标欧拉角。
  - `step(robot, world, command, grasped_object) -> int`
    - **说明**：执行一步动作，更新机器人状态。
  - `action_space` (property): Tuple(Discrete, Box)，如(动作类型, 参数值)。
  - `action_dim` (property): 2。
- **使用场景**：轮式/差动机器人移动。

### StretchGraspController
- **功能**：实现机械臂的抓取与释放。
- **典型属性**：
  - `arm_length` (float): 机械臂长度。
  - `is_grasping` (bool): 是否正在抓取。
- **主要方法**：
  - `get_action(command, robot=None) -> tuple`
    - **参数**：
      - `command` (list): ["grasp"/"release", 目标对象]。
    - **返回**：
      - (str, target_obj): 动作类型与目标。
  - `step(robot, world, command, grasped_object)`
    - **说明**：执行抓取/释放。
- **使用场景**：服务机器人、搬运机器人。

---

## 传感器 Sensors

### BaseSensor
- **功能**：所有传感器的抽象基类，定义观测数据接口。
- **典型属性**：
  - `type` (str): 传感器类型。
  - `on_robot` (bool): 是否安装在机器人上。
  - `data` (Any): 当前观测数据。
- **主要方法**：
  - `update() -> Any`
    - **说明**：更新并返回传感器观测。
  - `init(offset)`
    - **说明**：初始化传感器，offset为位姿偏移。

### VisionSensor
- **功能**：视觉传感器，支持RGB与深度模态。
- **典型属性**：
  - `modals` (list): 支持的模态（如["rgb", "depth"]）。
  - `resolution` (tuple): 图像分辨率。
  - `fov` (float): 视场角。
  - `frequency` (int): 采样频率。
- **主要方法**：
  - `update() -> dict`
    - **返回**：
      - `{ "rgb": np.ndarray, "depth": np.ndarray }`。
  - `init(offset)`
    - **说明**：在仿真中创建相机。
- **使用场景**：机器人视觉、SLAM、目标检测。

---

## 任务 Tasks

### BaseTask
- **功能**：所有任务的抽象基类，定义任务流程、评测与重置。
- **典型属性**：
  - `robots` (list): 参与任务的机器人。
  - `objects` (list): 任务相关物体。
  - `metrics` (dict): 评测指标。
  - `steps` (int): 当前步数。
- **主要方法**：
  - `step() -> (dict, dict, bool)`
    - **返回**：
      - 观测、信息、是否完成。
  - `is_done() -> bool`
    - **说明**：判断任务是否完成。
  - `individual_reset()`
    - **说明**：单独重置任务。
  - `get_observations() -> dict`
    - **说明**：获取当前观测。
  - `update_metrics()`
    - **说明**：更新评测信息。
  - `init(robots, objects)`
    - **说明**：初始化任务。

### NavigateTask
- **功能**：导航任务，支持路径规划、目标检测与多阶段目标。
- **典型属性**：
  - `start_points` (list): 起点坐标。
  - `goal_points` (list): 终点坐标。
  - `goal_threshold` (float): 到达判定阈值。
  - `max_steps` (int): 最大步数。
  - `map_path` (str): 地图文件路径。
- **主要方法**：
  - `step() -> (dict, dict, bool)`：执行导航。
  - `is_done() -> bool`：判断是否到达目标。
  - `get_shortest_path(start, goal) -> list`：获取最短路径。
  - `get_distance_to_goal() -> dict`：获取到目标距离。
- **使用场景**：室内导航、路径规划、任务型机器人。

---

## 机器人 Robots

### BaseRobot
- **功能**：所有机器人基类，封装控制器、传感器、动作与状态接口。
- **典型属性**：
  - `robot_config` (RobotConfig): 机器人配置。
  - `prim_path` (str): 机器人在仿真中的路径。
  - `usd_path` (str): 机器人USD模型路径。
  - `name` (str): 机器人名称。
  - `controllers` (list): 控制器实例列表。
  - `sensors` (list): 传感器实例列表。
  - `action_dim` (int): 动作维度。
  - `action_space` (Any): 动作空间。
- **主要方法**：
  - `apply_action(action)`：执行动作。
  - `get_joint_names() -> list[str]`：关节名列表。
  - `get_joint_positions() -> np.ndarray`：关节位置。
  - `get_world_pose() -> np.ndarray`：世界坐标。
  - `get_world_orientation() -> np.ndarray`：世界朝向。
  - `reset()`：重置机器人。
  - `init()`：初始化机器人。
- **使用场景**：移动机器人、机械臂、复合机器人。

### Stretch (示例)
- **功能**：轮式底盘+机械臂的复合机器人，支持移动与抓取。
- **典型属性/方法**：
  - `get_wheel_names() -> list[str]`：轮子关节名。
  - `get_arm_names() -> list[str]`：机械臂关节名。
  - `apply_action(action_instruct, world)`：执行复合动作。
  - `plan_length` (list): 路径长度记录。
- **使用场景**：服务机器人、家庭助理、实验平台。

---

## 典型用例

```python
from simulator.core.config import EnvConfig
from simulator.core.env import BaseEnv
from simulator.core.dataset import DatasetLoader
from simulator.agents.random import RandomAgent

# 加载数据集
loader = DatasetLoader(root_dir="resource/datasets/all_task",
                      scene_path="/data1/linmin/NPC/hssd_test_scene",
                      robot_path="tests/stretch/model/stretch.usd",
                      headless=False)

cfg = loader[0]

# 创建环境
env = BaseEnv(cfg)

# 创建智能体
agent = RandomAgent(config=None)

obs = env.reset()
while env.is_running:
    action = agent.act(obs)
    obs, info, reward, done = env.step([action])
    if done[0]:
        break
env.close()
```

---

## 联系与支持
如有疑问请联系 linm57@mail2.sysu.edu.cn 或访问 [文档主页](https://github.com/pzhren/InfiniteWorld)。
