#!/usr/bin/env python
#coding=utf-8

import rospy
import math
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped

class Buffer:
    def __init__(self, window, data_size):
        self._buffer = np.zeros((window, data_size))
        self._i = 0        

    def add(self, value):
        self._buffer[self._i] = value
        self._i+=1
        if self._i >= len(self._buffer):
            self._i = 0

    def median(self):
        return np.median(self._buffer, axis=0)

p1 = None
buffer = Buffer(5, 2)

def publish(dx, dy, stamp, pub):
    p = PointStamped()
    p.header.stamp = stamp
    p.point.x = dx
    p.point.y = dy
    pub.publish(p)

def odom_callback(msg):
    global p1

    if p1 is not None:
        dt = (msg.header.stamp - p1.header.stamp).to_sec()
        dx = (msg.pose.pose.position.x - p1.pose.pose.position.x) / dt
        dy = (msg.pose.pose.position.y - p1.pose.pose.position.y) / dt
        publish(dx, dy, msg.header.stamp, speed_pub)

        buffer.add([dx, dy])
        dx, dy = buffer.median()
        publish(dx, dy, msg.header.stamp, speed_filtered_pub)

    p1 = msg    

if __name__ == '__main__':
    rospy.init_node('path_move')
    rospy.loginfo('path_move started')
    rospy.Subscriber('/zed/odom', Odometry, odom_callback)

    speed_pub = rospy.Publisher('/dpos', PointStamped, queue_size=10)
    speed_filtered_pub = rospy.Publisher('/dpos_filtered', PointStamped, queue_size=10)

    rospy.spin()