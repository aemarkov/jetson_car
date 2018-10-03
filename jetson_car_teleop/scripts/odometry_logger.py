#!/usr/bin/env python
#coding=utf-8
# Логгирует одометрию (положение и ориентацию камеры) в файл

import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

path = Path()

def publish_path(odom): 
    global path
    path.header = odom.header
    pose = PoseStamped()
    pose.header = odom.header
    pose.pose = odom.pose.pose
    path.poses.append(pose)
    path_pub.publish(path)

def odom_cb(msg):
    if is_pub_path:
        publish_path(msg)

    if is_log_path:
        """TODO: log xyz rpy & quaternion to file"""

if __name__ == '__main__':   
    rospy.init_node('path_node')
    rospy.loginfo("Path node")

    is_pub_path = rospy.get_param('is_publish_path', True)
    is_log_path = rospy.has_param('logs_directory')
    logs_dir = None

    rospy.loginfo('Path publishing:        ' + ('enabled' if is_pub_path else 'disabled'))
    rospy.loginfo('Path logging:           ' + ('enabled' if is_log_path else 'disabled'))

    if is_log_path:
        logs_dir  = rospy.get_param('logs_directory')
        rospy.loginfo('Path logging directory: ' + logs_dir)

    if not is_pub_path and not is_log_path:
        rospy.logwarn("And why you've started me?")
        exit(0)

    odom_sub = rospy.Subscriber('/zed/odom', Odometry, odom_cb)

    if is_pub_path:
        path_pub = rospy.Publisher('/car_path', Path, queue_size=10)

    rospy.spin()
