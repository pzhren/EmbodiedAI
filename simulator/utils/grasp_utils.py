import numpy as np
from scipy.spatial.transform import Rotation as R


def transform_to_world(pos, orient, d):
    """
    Transform a 3D coordinate from the vehicle coordinate system to the world coordinate system.
    :param pos: 3D coordinate in the vehicle coordinate system (x, y, z)
    :param orient: Quaternion representing the orientation of the vehicle relative to the world coordinate system (w, x, y, z)
    :return: 3D coordinate in the world coordinate system (x, y, z)
    """
    
    rotation = R.from_quat(orient) # Convert quaternion to rotation matrix
    rotation_matrix = rotation.as_matrix() 
    world_pos = np.dot(rotation_matrix, d) + pos

    return world_pos

def transform_to_base(pos, orient, target, arm_length):
    """
    Calculate the robotic arm parameters based on the target position (requires moving to the correct position first).
    :param pos: Coordinate in the vehicle coordinate system (x, y, z)
    :param orient: Quaternion representing the orientation of the vehicle relative to the world coordinate system (w, x, y, z)
    :param target: Target position in the world coordinate system (x, y, z)
    :return: Parameters for the robotic arm [length, height difference]
    """

    rotation = R.from_quat(orient)
    rotation_matrix = rotation.as_matrix() 
    pos_transformed = np.dot(rotation_matrix.T, (target - pos))
    length = abs(-0.4 - pos_transformed[1])
    if length > arm_length:
        length = arm_length
    arg4grasp = [length,abs(pos_transformed[2] - 0.06)]
    return arg4grasp



def rpy2R(rpy): 
    """
    Convert the roll-pitch-yaw angles of the Stretch robot to a rotation matrix.
    :param rpy: [roll, pitch, yaw] in radians
    :return: Rotation matrix
    """
    rot_x = np.array([[1, 0, 0],
                    [0, math.cos(rpy[0]), -math.sin(rpy[0])],
                    [0, math.sin(rpy[0]), math.cos(rpy[0])]])
    rot_y = np.array([[math.cos(rpy[1]), 0, math.sin(rpy[1])],
                    [0, 1, 0],
                    [-math.sin(rpy[1]), 0, math.cos(rpy[1])]])
    rot_z = np.array([[math.cos(rpy[2]), -math.sin(rpy[2]), 0],
                    [math.sin(rpy[2]), math.cos(rpy[2]), 0],
                    [0, 0, 1]])
    R = np.dot(rot_z, np.dot(rot_y, rot_x))
    return R