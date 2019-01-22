#!/usr/bin/env python
#coding=utf-8

import math
import sys
import numpy as np
import rospy
import tf 
from geometry_msgs.msg import Point, Pose, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path

import quintic
import frenet
from rviz_helpers import LinesHelper
import msg_helpers

WORLD_FRAME = 'map'

REPLANE_PERIOD = 0.5
HORIZONT = 2
ROAD_WIDTH = 1

D_MIN = -ROAD_WIDTH/2.0
D_MAX =  ROAD_WIDTH/2.0

grid = None
pose = None
reference_path = None

def grid_callback(msg):
    global grid
    grid = msg
    rospy.loginfo('grid')

def pose_callback(msg):
    global pose
    pose = msg
    rospy.loginfo('pose')

def path_callback(msg):
    global reference_path
    reference_path = msg
    rospy.loginfo('path')

############################################################################


def get_horizont(path, start_index, horizont):
    s = 0 
    i = start_index

    while i < len(path.poses)-1 and s < horizont:
        p1 = path.poses[i].pose.position
        p2 = path.poses[i+1].pose.position
        dist = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
        s+=dist
        #rospy.loginfo('%0.2f, %0.2f' % (p1.x, p2.x))
        i+=1

    if i >= len(path.poses):
        return path.poses[start_index:]

    return path.poses[start_index:i+1] # include right 

def find_closest_point_index(path, position, start_index = 0):
    min_index = -1
    min_dist = sys.float_info.max

    for i in range(start_index, len(path.poses)):
        dist = (path.poses[i].pose.position.x - position.x)**2 + (path.poses[i].pose.position.y - position.y)**2
        if dist < min_dist:
            min_dist = dist
            min_index = i

    return min_index

def plan_path():
    if pose == None or reference_path == None:
        return 

    start_index = find_closest_point_index(reference_path, pose.pose.position)
    subpath = get_horizont(reference_path, start_index, HORIZONT)
    np_subpath = msg_helpers.path_poses_to_array(subpath)

    n = 50
    fp_c = np.zeros((n, 2)) 
    fp_min = np.zeros((n, 2)) 
    fp_max = np.zeros((n, 2)) 
    for i in range(n):
        fp_min[i] = [i*0.1, D_MIN]
        fp_c[i] = [i*0.1, 0]
        fp_max[i] = [i*0.1, D_MAX]

    
    fp_min = frenet.path_to_global(fp_min, np_subpath)
    fp_c = frenet.path_to_global(fp_c, np_subpath)
    fp_max = frenet.path_to_global(fp_max, np_subpath)

    road_borders_helper.lines([fp_min, fp_c, fp_max])


    #local_path = Path()
    #local_path.header.frame_id = WORLD_FRAME    
    #for p in subpath:
    #    local_path.poses.append(p)
    #path_pub.publish(local_path)

############################################################################

if __name__ == '__main__':
    rospy.init_node('motion_planner_node')

    listener = tf.TransformListener()
    rospy.Subscriber('/occupancy_grid', OccupancyGrid, grid_callback)
    rospy.Subscriber('/goal', PoseStamped, pose_callback)
    rospy.Subscriber('/reference_path', Path, path_callback)
    path_pub = rospy.Publisher('/local_path', Path, queue_size=1, latch=True)

    road_borders_helper = LinesHelper('/road_borders', [1,1,1,1])

    rate = rospy.Rate(1.0 / REPLANE_PERIOD)
    while not rospy.is_shutdown():
        plan_path()
        rate.sleep()