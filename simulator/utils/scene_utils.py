from lazyimport import lazyimport
import numpy as np
import math

lazyimport(globals(), """
    import omni.isaac.core.utils.prims as prim_utils
  """
)
def add_boundary_walls(width=80, height=60, wall_height=5, wall_thickness=1, center=(0, 0), env_id=0):
        """
        To add walls and ground around the boundaries of the scene and allow adjusting the center position of the walls.
        
        Args:
        - width (float): The boundary size of the scene in the X direction, i.e., the length of the wall in the horizontal direction.
        - height (float): The boundary size of the scene in the Y direction, i.e., the length of the wall in the vertical direction.
        - wall_height (float): The height of the wall.
        - wall_thickness (float): The thickness of the wall.
        - center (tuple): The center position of the wall, formatted as (x, y).
        """
    # 墙体尺寸
        wall_x_size = width + 2 * wall_thickness   # X方向的墙体长度
        wall_y_size = height + 2 * wall_thickness  # Y方向的墙体长度

        # 中心位置的坐标
        center_x, center_y = center

        # 创建上方围墙（水平长方体）
        prim_utils.create_prim(
            prim_path="/World/Wall_Top"+str(env_id),
            prim_type="Cube",
            translation=(center_x, center_y + height / 2 + wall_thickness / 2, wall_height / 2),
            scale=(wall_x_size, wall_thickness, wall_height)
        )

        # 创建下方围墙（水平长方体）
        prim_utils.create_prim(
            prim_path="/World/Wall_Bottom"+str(env_id),
            prim_type="Cube",
            translation=(center_x, center_y - height / 2 - wall_thickness / 2, wall_height / 2),
            scale=(wall_x_size, wall_thickness, wall_height)
        )

        # 创建左侧围墙（垂直长方体）
        prim_utils.create_prim(
            prim_path="/World/Wall_Left"+str(env_id),
            prim_type="Cube",
            translation=(center_x - width / 2 - wall_thickness / 2, center_y, wall_height / 2),
            scale=(wall_thickness, wall_y_size, wall_height)
        )

        # 创建右侧围墙（垂直长方体）
        prim_utils.create_prim(
            prim_path="/World/Wall_Right"+str(env_id),
            prim_type="Cube",
            translation=(center_x + width / 2 + wall_thickness / 2, center_y, wall_height / 2),
            scale=(wall_thickness, wall_y_size, wall_height)
        )

        # 创建地面
        prim_utils.create_prim(
            prim_path="/World/Wall_Bottom_Surface"+str(env_id),
            prim_type="Cube",
            translation=(center_x, center_y, -0.5),  # 地面位置低于围墙底部
            scale=(wall_x_size, wall_y_size, wall_thickness)  # 地面的尺寸
        )

def compute_enclosing_square(aabb):
    """
    根据给定的 AABB 计算可以包围它的正方形的中心和边长。
    
    Args:
    - aabb (tuple): AABB 的坐标，格式为 (xmin, ymin, zmin, xmax, ymax, zmax)。
    
    Returns:
    - center (tuple): 正方形的中心坐标 (x_center, y_center)。
    - side_length (float): 正方形的边长。
    """
    xmin, ymin, zmin, xmax, ymax, zmax = aabb

    # 计算正方形的中心坐标
    x_center = (xmin + xmax) / 2
    y_center = (ymin + ymax) / 2

    # 计算正方形的边长
    # side_length = max(xmax - xmin, ymax - ymin)
    # adjusted_side_length = math.ceil(side_length / 5) * 5
    width = xmax-xmin
    height = ymax-ymin
    adjusted_side_width = math.ceil(width/5)*5
    adjusted_side_height = math.ceil(height/5)*5

    return (x_center, y_center), adjusted_side_width, adjusted_side_height
