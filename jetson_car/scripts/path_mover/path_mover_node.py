#!/usr/bin/env python
#coding=utf-8

import math
import numpy as np
import sys

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from tf.transformations import euler_from_quaternion

import msg_helpers
from rviz_helpers import RvizHelpers

###############################################################################

RADIUS = 0.3             # Радиус просмотра вперед
P_COEF = 2.5             # Коэффциент управления

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

# Находит индекст точки на пути, ближайшей к заданной
def find_closest_point_index(path, position):
    min_index = -1
    min_dist = sys.float_info.max

    for i in range(len(path.poses)):
        dist = (path.poses[i].pose.position.x - position.x)**2 + (path.poses[i].pose.position.y - position.y)**2
        # <= - потому что важно найти самую последнюю ближашую точку
        if dist <= min_dist:
            min_dist = dist
            min_index = i

    return min_index

# Находит точку на пути, расположенную впереди (по траектории) заданной, 
# не ближе, чем радиус
def find_ahead_point(path, position, radius):
    index0 = find_closest_point_index(path, position)
    index = index0
    r2 = radius**2

    while index < len(path.poses):
        dist = (path.poses[index].pose.position.x - position.x)**2 + (path.poses[index].pose.position.y - position.y)**2
        if dist >= r2:
            return (index0, path.poses[index].pose.position)

        index+=1

    return (index0, path.poses[-1].pose.position)

# получение требуемой траектории
def path_callback(msg):
    global path
    path = msg
    #rospy.loginfo(len(path.poses))

# получение текущего положения машинки
last_path_point_index = 0
def pose_callback(msg):
    global last_path_point_index

    if path is None:
        return

    # Текущее положение
    position = msg.pose.position
    orientation =  euler_from_quaternion(msg_helpers.quaterion_to_array(msg.pose.orientation))[2]

    # определение ближайшего пересечения траектории с окружностью
    # intersection - устаревшее, исходное название. Так-то теперь это точка на траектории где-то впереди
    index, intersection = find_ahead_point(path, position, RADIUS)

    # отрисова окружности поиска для оладки
    rviz.circle(position, RADIUS)
    
    # Отрисовка пересечений
    rviz.intersection(position, path.poses[index].pose.position)

    if index >= len(path.poses)-1:
        rospy.loginfo("End of the path")
        send_command(0,0)
        return    

    # управленияе взависимости от угла отклонения текущего
    # курса от курса на пересечение
    bearing = calc_bearing(msg_helpers.point_to_array(position), orientation, msg_helpers.point_to_array(intersection))
    rot = P_COEF * bearing

    if abs(rot) > 1:
        rot = math.copysign(1, rot)

    send_command(1, rot)


#########################################################

if __name__ == '__main__':
    rospy.init_node('path_move')
    rospy.loginfo('path_move started')

    RADIUS = rospy.get_param('~radius', RADIUS)    
    P_COEF = rospy.get_param('~P_COEF', P_COEF)

    rospy.loginfo('RADIUS: ', RADIUS)
    rospy.loginfo('P_COEF: ', P_COEF)

    commands_pub = rospy.Publisher('joy', Joy, queue_size = 100)
    rospy.Subscriber('/reference_path', Path, path_callback)
    rospy.Subscriber('/zed/pose', PoseStamped, pose_callback)
    rviz = RvizHelpers('/circle', '/intersect')
    
    rospy.spin()