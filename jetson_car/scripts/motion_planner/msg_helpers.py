import numpy as np
from geometry_msgs.msg import Point

def point_to_array(p):
    return np.array([p.x, p.y])    

def path_poses_to_array(poses):
    arr = np.zeros((len(poses), 2))
    for i, pose in enumerate(poses):
        arr[i] = point_to_array(pose.pose.position)

    return arr

def array_to_point(p):
    return Point(p[0], p[1], 0)