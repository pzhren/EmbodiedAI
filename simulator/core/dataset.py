from simulator.core.config import *
import random
from simulator.utils.config_utils import merge_config
import copy
import json
class DatasetLoader:
    def __init__(self, root_dir, scene_path, robot_path, shuffle=False, task_type="single", device_id=0, headless=True):
        assert task_type in ["single", "multi"]
        self.type = task_type
        self.root_dir = root_dir
        self.scene_path = scene_path
        self.robot_path = robot_path
        self.shuffle = shuffle
        self.device_id = device_id
        self.headless = headless
        self.config_paths = self._collect_scene_paths()
        self.cache = {}  # 可选：缓存已加载的 envconfigs（避免重复加载）

    def _collect_scene_paths(self):
        scene_paths = []
        for subdir in os.listdir(self.root_dir):
            subdir_path = os.path.join(self.root_dir, subdir)
            if os.path.isdir(subdir_path):
                # 遍历每个子目录下的子目录，寻找process_config.json
                for inner_subdir in os.listdir(subdir_path):
                    inner_subdir_path = os.path.join(subdir_path, inner_subdir)
                    if os.path.isdir(inner_subdir_path):
                        config_path = os.path.join(inner_subdir_path, 'processed_config.json')
                        if os.path.exists(config_path):
                            scene_paths.append(config_path)
        if self.shuffle:
            random.shuffle(scene_paths)
        return scene_paths

    def __getitem__(self, index):
        if index < 0 or index >= len(self.config_paths):
            raise IndexError("Index out of range.")
        
        if index in self.cache:
            return self.cache[index]

        config_path = self.config_paths[index]
        with open(config_path, 'r') as f:
            config_data = json.load(f)

        env_config = self.process(config_data, config_path)
        self.cache[index] = env_config
        return env_config

    def __len__(self):
        return len(self.config_paths)
    
    def _find_png_files(self, directory):
        """查找给定目录下所有的 .jpg 文件并返回它们的路径列表"""
        jpg_files = []
        for root, _, files in os.walk(directory):
            for file in files:
                if file.lower().endswith('.png'):
                    jpg_files.append(os.path.join(root, file))
        return jpg_files
    
    def process(self, config_data, config_path):
        # TODO: 替换为你自己的转换逻辑
        config_dict = {
            "env_num":1,
            "sim":{
                "device":self.device_id,
                "headless":self.headless,
                "physics_frequency":60,
                "render_frequency":60,
                "hide_ui":False,
            },
            "scene":{
                "type": "Interactive_Scene",
                "scene_file":os.path.join(self.scene_path, config_data["Scene"], config_data["Scene"]+".usd"),
                "use_floor_plane":False,
                "add_wall":True
            },
            "task":{
                "type": "NavigateTask",
                "robots":self.make_robot_config(config_data["Start"]),
                "task_instruction": config_data["Task instruction"],
                "goal_points": config_data["End"],
                "start_points": config_data["Start"],
                "reference_path": config_data["Reference path"],
                "goal_image_path": self._find_png_files(os.path.dirname(config_path)),
                "map_path":self._find_png_files(os.path.join(self.scene_path, config_data["Scene"]))[0],
                "task_path":config_path,
            }


        }
        if self.type == "multi":
            config_dict["task"]["metrics"]=[
                    {
                        "type":"NE",
                        "name":"NE"
                    },
                    {
                        "type":"SPL",
                        "name":"SPL"
                    },
                    {
                        "type": "PL",
                        "name": "PL"
                    },
                    {
                        "type": "CR",
                        "name": "CR"
                    }
                ]
        else:
            config_dict["task"]["metrics"]=[
                    {
                        "type":"NE",
                        "name":"NE"
                    },
                    {
                        "type":"SPL",
                        "name":"SPL"
                    },
                    {
                        "type": "PL",
                        "name": "PL"
                    },
                ]

        config_dict = merge_config(
            self.default_config.dict(), 
            config_dict,
            inplace = True, 
            verbose = True
        )
        return Config(**config_dict)

    def reset_cache(self):
        self.cache.clear()
   
    def make_robot_config(self, start_point):
        # robot_config = {}
        def make_init_config(robot_prim_path, robot_name, init_pos):
            init_config = {
                "type": "Stretch",
                "name": "stretch",
                "prim_path": robot_prim_path,
                "position": init_pos,
                "use_position": True,
                "usd_path": self.robot_path,
                "create_robot": True,
                "controllers":
                [
                    {
                        "type": "PositionController"
                    }
                ],
                "sensors":
                [
                    {
                        "type":"VisionSensor",
                        "on_robot": True,
                        "name":robot_name+"_front_camera",
                        "prim_path":robot_prim_path+"/camera_color_frame/rgb_camera",
                        "modals":["rgb","depth"],
                        # "orientation":[1,0,0,0],
                        "orientation":[0.707,-0.707,0,0]
                        # "position":[0,0,0.8]
                    },
                    {
                        "type":"VisionSensor",
                        "name":robot_name+"_left_camera",
                        "on_robot": True,
                        "prim_path":robot_prim_path+"/camera_color_frame/rgb_camera_l",
                        "modals":["rgb","depth"],  
                        # "orientation":[0.707,0,0,0.707],
                        "orientation":[0.5,-0.5,0.5,0.5],
                        # "position":[0,0,0.8]
                    },
                    {
                        "type":"VisionSensor",
                        "on_robot": True,
                        "name":robot_name+"_right_camera",
                        "prim_path":robot_prim_path+"/camera_color_frame/rgb_camera_r",
                        "modals":["rgb","depth"],  
                        # "orientation":[0.707,0,0,-0.707],
                        "orientation":[0.5,-0.5,-0.5,-0.5],
                        # "position":[0,0,0.8]
                    }
                ]

            }
            return init_config
        robotconfig = []
        if self.type == "multi":
            robotconfig = [make_init_config("/World/Robot0","robot0",init_pos=start_point[0][0]),
                        make_init_config("/World/Robot1","robot1",init_pos=start_point[0][1])]
        elif self.type == "single":
            robotconfig=[make_init_config("/World/Robot0","robot0",init_pos=start_point[0])]
        return robotconfig

    @property
    def default_config(self):
        """
        Returns:
            config: Default configuration for this environment. 
        """
        #Simulator kwargs
        default_sim = {
            "height": 720,
            "width": 1280,
            "device": 0, 
            "physics_frequency": 60,
            "render_frequency": 60,
            "headless": True,
        }

        #Scene kwargs
        default_scene = {
            "scene_file": None
        }
        default_task = {}
        default_npc = {}
        return Config(
            name="default",
            sim=SimulatorConfig(**default_sim),
            scene=SceneConfig(**default_scene),
            task=TaskConfig(**default_task),
            npcs=[NPCConfig(**default_npc)]
        )


