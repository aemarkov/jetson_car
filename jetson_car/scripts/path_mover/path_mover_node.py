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

radius = 0.3             # Радиус просмотра вперед
stop_cnt = 5             # Количество оставшихся точек до конца траектории, при которых можно остановится
points_lookup_count = 10 # В каком диапазоне нужно искать следующие точки (от последней)
k = 1.5                  # Коэффциент управления

path = None
###############################################################################

def __rot_matrix(angle):
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[c, -s],
                     [s,  c]])

def __rot_vector(vec, angle):
    return np.matmul(__rot_matrix(angle), vec)

def __vec_of_angle(angle):
    return __rot_vector(np.array([1,0]), angle)

# Вычисляет угол отклонения между текущим курсом и пересечением
def calc_bearing(pos, orientation, intersect):  
    vec_to_intersect =  intersect - pos
    orientation_vec = __vec_of_angle(orientation)
    bearing = math.atan2(np.cross(orientation_vec, vec_to_intersect), np.dot(vec_to_intersect, orientation_vec))
    return bearing

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
    index, intersection = intersections_finder.find(path,position, radius, 0.01, 
                                                    last_path_point_index, 
                                                    points_lookup_count)

    last_path_point_index = index

    print('>>>', last_path_point_index, intersection)

    if len(path.poses) - last_path_point_index <= stop_cnt:
        send_command(0,0)
        return

    # отрисова окружности поиска для оладки
    rviz.circle(position, radius)

    if intersection is None:
        rospy.logwarn('Too far from trajectory')
        send_command(0, 0)
        return
    
    # Отрисовка пересечений
    rviz.intersection(position, intersection)

    # управленияе взависимости от угла отклонения текущего
    # курса от курса на пересечение
    bearing = calc_bearing(position, orientation, intersection)
    rot = k * bearing
    #print(math.degrees(bearing), rot)
    if abs(rot) > 1:
        rot = math.copysign(1, rot)

    send_command(1, rot)


#########################################################

if __name__ == '__main__':
    rospy.init_node('path_move')
    rospy.loginfo('path_move started')



    commands_pub = rospy.Publisher('joy', Joy, queue_size = 100)
    rospy.Subscriber('/path', Path, path_callback)
    rospy.Subscriber('/zed/odom', Odometry, odom_callback)
    rviz = RvizHelpers('/circle', '/intersect')
    
    rospy.spin()