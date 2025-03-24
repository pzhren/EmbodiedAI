import isaacsim
import carb
import math
from omni.isaac.kit import SimulationApp
from typing import Any, Callable, Dict, List, Optional
from simulator.core.config import SimulatorConfig, PrimConfig
from simulator.utils.log_utils import create_module_log
from simulator.core.robot import BaseRobot
from simulator.core.scene import BaseScene
from transformations import quaternion_from_euler
import json



from lazyimport import lazyimport
lazyimport(globals(), """
    from omni.isaac.core import World
    import omni.isaac.core.utils.prims as prim_utils
    from omni.isaac.core.prims import XFormPrim
    from omni.isaac.nucleus import get_assets_root_path
    from simulator.utils.scene_utils import add_boundary_walls
    from simulator.utils.scene_utils import compute_enclosing_square
    from simulator.core.itemlookup import ItemLookup
  """
)

def launch_app(app_config:dict, webrtc=False, native=False):
    """
    Launch the isaac sim app.
    """
    simulation_app = SimulationApp(app_config)
        
    if webrtc:
        from omni.isaac.core.utils.extensions import enable_extension  

        simulation_app.set_setting('/app/window/drawMouse', True)
        simulation_app.set_setting('/app/livestream/proto', 'ws')
        simulation_app.set_setting('/app/livestream/websocket/framerate_limit', 60)
        simulation_app.set_setting('/ngx/enabled', False)
        enable_extension('omni.services.streamclient.webrtc')

    elif native:
        from omni.isaac.core.utils.extensions import enable_extension  

        simulation_app.set_setting("/app/window/drawMouse", True)
        simulation_app.set_setting("/app/livestream/proto", "ws")
        simulation_app.set_setting("/app/livestream/websocket/framerate_limit", 120)
        simulation_app.set_setting("/ngx/enabled", False)
        enable_extension("omni.kit.livestream.native")

    return simulation_app

class Simulator():
    def __init__(self, config:SimulatorConfig, webrtc=False, native=False):
        
        self.phy_dt = 1./config.physics_frequency 
        self.render_dt = 1./config.render_frequency
        self.headless = config.headless
        self.height = config.height
        self.width = config.width
        self.hide_ui = config.hide_ui
        self.device = config.device
        self.app_config = {
            'headless': self.headless, 
            'height': self.height, 
            'width': self.width, 
            'physics_gpu': self.device,
            'active_gpu': self.device,
            }
        try:
            self._simulation_app = launch_app(self.app_config, webrtc=webrtc, native=native)
        except Exception as e:
            log.error("Can not lanuch isaac sim app.")
            raise e

        self._world = World(physics_dt=self.phy_dt, rendering_dt=self.render_dt, stage_units_in_meters=1.0)
        self._scene = self._world.scene
        self._stage = self._world.stage
        self._resource_path = "./resource"
        self._warm_up()
        self.is_playing = self._world.is_playing()

    def _warm_up(self, steps=20, render=True):
        """
        Warm up the simulator.
        """
        for _ in range(steps):
            self._world.step(render=render)
    
    def add_tasks(self, tasks):
        for config in configs:
            task = create_task(config, self._scene)
            self._world.add_task(task)

        self._world.reset()
        self._warm_up()
    
    def import_robot(self, robot, offset=[0,0,0]):
        assert self.is_playing==False
        assert isinstance(robot, BaseRobot)
        # prim_utils
        prim_utils.create_prim(
            prim_type = "Xform",
            prim_path = robot.prim_path,
            usd_path = robot.usd_path,
            translation = [x + y for x, y in zip(robot.position, offset)],
            orientation = robot.orientation,
            scale = robot.scale,
            semantic_label = robot.name
            # orientation=[0.70711,0.0,0.0,-0.70711]
            )
        pass

    # def import_sensor(self, sensor):
    #     assert self._world.is_playing==False
    #     assert isinstance(sensor, BaseSensor)
    #     if sensor.type=="vision":
    #         prim_utils.create_prim(
    #             prim_type = "Camera",
    #             prim_path = sensor.prim_path,
    #             position = sensor.postion,
    #             orientation = sensor.orientation,
    #         )

    #     return sensor

    def import_scene(self, scene, offset=[0,0,0], scene_id=0):
        assert self.is_playing==False
        assert isinstance(scene, BaseScene)

        if scene._use_floor_plane:
            assets_root_path = get_assets_root_path()
            self._world.scene.add_default_ground_plane()
            if not scene._floor_plane_visible:
                prim_utils.set_visibility(prim_path="/World/defaultGroundPlane", visible=False)
        
        if scene._use_sky_box:
            prim_utils.create_prim(
                prim_path="/World/defaultSky",
                prim_type = "DomeLight",
                attributes={
                    "inputs:intensity": 1500,
                    # "inputs:radius":1.0,
                    "inputs:texture:file": f"{self._resource_path}/background/sky/sky.jpg"
                }
            )

        for k,v in scene.scene_prim_dict.items():
            # pass
           v.init()

        if scene._add_wall:
            scene_prim = scene.scene_prim_dict["Scene"]
            scene_aabb = scene_prim.get_aabb()
            center,width,height = compute_enclosing_square(scene_aabb)
            add_boundary_walls(width=width, height=height, wall_height=5, wall_thickness=0.5,center=center, env_id=scene_id)
        

    def extract_target_ids(self, json_path):
        """
        Reads the JSON file from the given path and returns a list of target IDs.
        
        :param json_path: Path to the JSON file.
        :return: List of target IDs.
        """
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        # 获取 "Target" 字段中的所有子项，提取每个子项的第一个元素
        target_ids = [item[0] for item in data.get("Target", []) if item]
        return target_ids
    

    def find_object_by_id(self, scene, objs_id):
        root_path ="/Scene0/floorplan/furniture/"
        # 获取scene的prim_path
        for _, scene_item in scene.scene_prim_dict.items():
            # 获取场景的所有物品名字，prim_dict的键为prim名字，值为对应的prim
            child = prim_utils.get_prim_children(scene_item.prim)
            child = prim_utils.get_prim_children(child[0])
            prim_list = []
            prim_dict = {}
            for prim in child:
                prim_list.extend(prim_utils.get_prim_children(prim))
            for prim in prim_list:
                prim_dict[prim.GetName()]=prim
            
            xform_prims = []
            
            for obj_id in objs_id:
                # 检查原始ID是否存在，否则尝试添加下划线前缀
                modified_id = obj_id
                if modified_id not in prim_dict:
                    modified_id = "_" + modified_id[1:]

                # 构造完整路径
                obj_path = root_path + modified_id
                
                # 获取对象属性
                obj_attrs = {}
                for attr in prim_dict[modified_id].GetAttributes():
                    obj_attrs[attr.GetName()] = attr.Get()

                r, p, y = obj_attrs["xformOp:rotateXYZ"]
                
                quaternion = quaternion_from_euler(
                    math.radians(r),
                    math.radians(p),
                    math.radians(y)
                )
                
                # 创建XFormPrim并添加到列表
                xform_prim = XFormPrim(
                    prim_path=obj_path,
                    translation=obj_attrs["xformOp:translate"],
                    orientation=quaternion
                )
                xform_prims.append(xform_prim)

            return xform_prims
        
    
    def find_object_around(self, scene, pos):
        self.hssd_item = ItemLookup("/data1/lfwj/linmin_embodiedAI/semantics_objects.csv")
        for _, scene_item in scene.scene_prim_dict.items():
            children = prim_utils.get_prim_children(scene_item.prim)
            if not children:
                continue
            grandchildren = prim_utils.get_prim_children(children[0])
            
            prim_dict = {}
            for prim in grandchildren:
                for child_prim in prim_utils.get_prim_children(prim):
                    prim_dict[child_prim.GetName()] = child_prim
        
             # 对所有物品都进行计算位置
            all_obj_dict = {
                every_id: attr.Get()
                for every_id, prim in prim_dict.items()
                for attr in prim.GetAttributes()
                if attr.GetName() == "xformOp:translate"
            }
        
            around_set = set()
            # 输出当前位置2m范围内的所有物品
            for Id in  all_obj_dict.keys():
                if math.dist([all_obj_dict[Id][0], all_obj_dict[Id][2]],pos) < 2:
                    name = self.hssd_item.get_item_name_by_id(Id)
                    if name!='':
                        around_set.add(name)
            
            # 将集合转换为列表
            return list(around_set)


                
    def play(self):
        return self._world.play()

    def step(self, render:bool=True) -> dict[str, Any]:
        return self._world.step(render=render)

    def reset(self):
        self._world.reset()

    def close(self):
        self._simulation_app.close()

    @property
    def current_tasks(self):
        return self._world._current_tasks

    @property
    def is_running(self) -> bool:
        return self._simulation_app.is_running()

   
    
  