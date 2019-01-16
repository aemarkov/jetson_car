#!/usr/bin/env python
#coding=utf-8

import rospy
from geometry_msgs.msg import Point, Pose, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import tf 

REPLANE_PERIOD = 0.5

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

def find_closest_point_on_path(path, pose):
    point = pose.pose.position
    l = [(path.poses[i].pose.position, i) for i in range(len(path.poses))]
    return min(l, key = lambda p: ((p[0].x - point.x)**2 - (p[0].y - point.y)))[1]

def plan_path():
    if grid == None or pose == None or reference_path == None:
        return 

    rospy.loginfo('wtf')

if __name__ == '__main__':
    rospy.init_node('motion_planner_node')

    listener = tf.TransformListener()
    rospy.Subscriber('/occupancy_grid', OccupancyGrid, grid_callback)
    rospy.Subscriber('/goal', PoseStamped, pose_callback)
    rospy.Subscriber('/reference_path', Path, path_callback)
    path_pub = rospy.Publisher('/local_path', Path, queue_size=1, latch=True)

    rate = rospy.Rate(1.0 / REPLANE_PERIOD)
    while not rospy.is_shutdown():
        plan_path()
        rate.sleep()