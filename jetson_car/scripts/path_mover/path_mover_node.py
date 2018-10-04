#!/usr/bin/env python
#coding=utf-8

import math
import numpy as np

import rospy
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Joy

import msg_helpers
import intersections_finder
from rviz_helpers import RvizHelpers

###############################################################################

path = None
radius = 0.3        # Радиус просмотра вперед
stop_radius = 0.3   # Радиус детектирования конечной точки
k = 1               # Коэффциент управления

###############################################################################

def is_close_to_point(pos, point, radius):
    return np.linalg.norm(point - pos) <= radius

def send_command(forward, rot):
    joy = Joy()
    joy.axes = [0, forward, rot]
    commands_pub.publish(joy)

###############################################################################

# получение требуемой траектории
def path_callback(msg):
    global path
    path = msg

# получение текущего положения машинки
def odom_callback(msg):
    if path is None:
        return

    # Текущее положение
    position = msg_helpers.point_to_array(msg.pose.pose.position)
    orientation =  euler_from_quaternion(msg_helpers.quaterion_to_array(msg.pose.pose.orientation))[2]    
        
    if is_close_to_point(position, msg_helpers.point_to_array(path.poses[-1].pose.position), stop_radius):
        send_command(0, 0)
        return

    # определение ближайшего пересечения траектории с окружностью
    intersects = intersections_finder.find_path_intersections(path, position, radius, 0.01)
    if len(intersects) == 0:
        rospy.logwarn('Too far from trajectory')
        send_command(0, 0)
        return

    int_pos, bearing = intersections_finder.find_closest_intersect(position, orientation,  intersects)
        
    # управленияе взависимости от угла отклонения текущего
    # курса от курса на пересечение
    rot = k * bearing
    if abs(rot) > 1:
        rot = math.copysign(1, rot)

    send_command(0.7, rot)

    # отрисова окружности поиска для оладки
    rviz.circle(position, radius)
    rviz.intersection(position, int_pos)


#########################################################

if __name__ == '__main__':
    rospy.init_node('path_move')
    rospy.loginfo('path_move started')
         
    commands_pub = rospy.Publisher('joy', Joy, queue_size = 100)
    rospy.Subscriber('/path', Path, path_callback)
    rospy.Subscriber('/zed/odom', Odometry, odom_callback)

    rviz = RvizHelpers('circle', 'intersect')

    rospy.spin()
    rospy.loginfo('Stopping...')
    send_command(0, 0)