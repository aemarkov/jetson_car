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
radius = 0.5             # Радиус просмотра вперед
stop_radius = 0.2        # Радиус детектирования конечной точки
k = 1                    # Коэффциент управления
points_lookup_count = 10 # В каком диапазоне нужно искать следующие точки (от последней)

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
last_path_point_index = 0
def odom_callback(msg):
    global last_path_point_index

    if path is None:
        return

    # Текущее положение
    position = msg_helpers.point_to_array(msg.pose.pose.position)
    orientation =  euler_from_quaternion(msg_helpers.quaterion_to_array(msg.pose.pose.orientation))[2]

    # определение ближайшего пересечения траектории с окружностью
    intersection = intersections_finder.find_path_intersection(path,position, radius, 0.01, 
                                                              last_path_point_index, points_lookup_count)

    print('>>>', intersection)
    # отрисова окружности поиска для оладки
    rviz.circle(position, radius)

    # Отрисовка пересечений
    if intersection[1] != None:
        rviz.intersection(position, intersection[1])
    
    return

    #if len(intersects) == 0:
    #    rospy.logwarn('Too far from trajectory')
    #    send_command(0, 0)
    #    return

    #last_path_point_index, int_pos, bearing = intersections_finder.find_closest_intersect(position,orientation,  intersects)
        
    # управленияе взависимости от угла отклонения текущего
    # курса от курса на пересечение
    #rot = k * bearing
    #print(math.degrees(bearing), rot)
    #if abs(rot) > 1:
    #    rot = math.copysign(1, rot)

    #send_command(1, rot)


#########################################################

if __name__ == '__main__':
    rospy.init_node('path_move')
    rospy.loginfo('path_move started')



    commands_pub = rospy.Publisher('joy', Joy, queue_size = 100)
    rospy.Subscriber('/path', Path, path_callback)
    rospy.Subscriber('/zed/odom', Odometry, odom_callback)
    rviz = RvizHelpers('/circle', '/intersect')
    
    rospy.spin()