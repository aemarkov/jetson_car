import numpy as np
from geometry_msgs.msg import Point

def quaterion_to_array(q):
    return np.array([q.x, q.y, q.z, q.w])

def point_to_array(p):
    return np.array([p.x, p.y])    

def array_to_point(p):
    return Point(p[0], p[1], 0)