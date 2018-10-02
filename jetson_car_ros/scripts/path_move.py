#!/usr/bin/env python
#coding=utf-8

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Joy

import math
import numpy as np

circle_pub = None
intersect_pub = None

circle_marker = Marker()
circle_marker.type = Marker.LINE_STRIP
circle_marker.header.frame_id = 'map'
circle_marker.scale.x = 0.01
circle_marker.scale.y = 0.01
circle_marker.scale.z = 0.01
circle_marker.color.b = 1
circle_marker.color.a = 1


intersect_marker = Marker()
intersect_marker.type = Marker.LINE_LIST
intersect_marker.header.frame_id = 'map'
intersect_marker.scale.x = 0.01
intersect_marker.scale.y = 0.01
intersect_marker.scale.z = 0.01
intersect_marker.color.r = 1
intersect_marker.color.g = 1
intersect_marker.color.a = 1

# Публикует маркер кольца
def publish_circle(center, r):
    steps = 20
    circle_marker.points = []
    for i in range(steps+1):
        p =  Point()
        angle = 2 * math.pi / steps * i
        p.x = center[0] + math.cos(angle)*r
        p.y = center[1] + math.sin(angle)*r
        p.z = 0
        circle_marker.points.append(p)

    circle_pub.publish(circle_marker)

# Нормирует вектор
def norm(vec):
    return vec / np.linalg.norm(vec)

# проверяет, что точка на прямой лежит в пределах отрезка
# a - точка
# A, B - границы отрезка
def check_bounds(p, A, B):
    x1 = min(A[0], B[0])
    x2 = max(A[0], B[0])
    y1 = min(A[1], B[1])
    y2 = max(A[1], B[1])
    
    return p[0] >= x1 and p[0] <= x2 and p[1] >= y1 and p[1] <= y2

# Нахождение точек пересечения между окружностью и отрезком
# p1,p2  - границы отрезка
# O      - центр окружности
# r      - радиус окружности
# eps    - точность проверки 
def find_intersections(p1, p2, O, r, eps):
    # прямая представлена в виде
    # (x - p1.x)/(p2.x - p1.x) = (y - p1.y)/(p2.y - p1.y)
    # представим ее в виде
    # ax + by + c = 0
    px = p2[0] - p1[0]
    py = p2[1] - p1[1]
    a = py
    b = -px
    c = px*p1[1] - py*p1[0]        
    
    x0 = O[0]
    y0 = O[1]
    
    # расстояние от центра окружности (точка O) до прямой
    d = abs(a*x0 + b*y0 + c)/math.sqrt(a*a + b*b)    
    
    # координаты ближайшей к центру точки на прямой (точка C)
    x = (b*(b*x0 - a*y0) - a*c)/(a*a + b*b)
    y = (a*(-b*x0 + a*y0) - b*c)/(a*a + b*b)
       

    if abs(d-r) < eps:
        # Одно пересечение
        A = [x, y]
        if check_bounds(A, p1, p2):
            return [A]
        else:
            return []
    
    if d > r:
        # Пересечений нет
        return []
    
    # иначе 2 пересечение
    
    # Расстояние от точки D до точек пересечения (A, B)
    # (одинаково в обе стороны)
    l = math.sqrt(r*r - d*d)
    
    # Отложим расстояние l от точки C вдоль и против нормированного направляющего вектора
    # прямой p(px, py):
    p = norm([px, py])
    C = [x, y]
    A = C - l*p
    B = C + l*p    
        
    # Проверка границ отрезка
    points = []
    if check_bounds(A, p1, p2):
        points.append(A)
        
    if check_bounds(B, p1, p2):
        points.append(B)        
    
    return points


# Нахождение точек пересечения между окружностью и ломаной, заданной
# последовательностью точек
# path   - траектория (ломаная)
# O      - центр окружности
# r      - радиус окружности
# eps    - точность проверки 
def find_path_intersections(path, O, r, eps):        
    intersetcs = []
    for i in range(len(path.poses)-1):        
        p = find_intersections(point_to_array(path.poses[i].pose.position),
                               point_to_array(path.poses[i+1].pose.position),
                               O, r, eps)
        intersetcs.extend(p)
    
    return intersetcs


# угол между вектором и осью x
def vector_angle(vec):
    return math.atan2(vec[1], vec[0])

# находит пересечение, угол на который меньше всего отличатеся от текущего курса
def find_closest_intersect(pos, orientation, intersects):    
    return  min(map(lambda x: (x, vector_angle(x - pos) - orientation), intersects), key=lambda x: abs(x[1]))


def is_close_to_point(pos, point, radius):
    return np.linalg.norm(point - pos) <= radius

def send_command(forward, rot):
    joy = Joy()
    joy.axes = [0, forward, rot]
    commands_pub.publish(joy)

######################################################

def quaterion_to_array(q):
    return np.array([q.x, q.y, q.z, q.w])

def point_to_array(p):
    return np.array([p.x, p.y])    

def array_to_point(p):
    return Point(p[0], p[1], 0)

######################################################


path = None
radius = 0.3        # Радиус просмотра вперед
stop_radius = 0.3   # Радиус детектирования конечной точки
k = 1               # Коэффциент управления

# получение требуемой траектории
def path_callback(msg):
    global path
    path = msg
    rospy.loginfo('path')

# получение текущего положения машинки
def odom_callback(msg):
    if path is None:
        return

    # Текущее положение
    position = point_to_array(msg.pose.pose.position)
    orientation =  euler_from_quaternion(quaterion_to_array(msg.pose.pose.orientation))[2]    
        
    if is_close_to_point(position, point_to_array(path.poses[-1].pose.position), stop_radius):
        send_command(0, 0)
        return

    # определение ближайшего пересечения траектории с окружностью
    intersects = find_path_intersections(path, position, radius, 0.01)
    if len(intersects) == 0:
        rospy.logwarn('Too far from trajectory')
        send_command(0, 0)
        return

    int_pos, bearing = find_closest_intersect(position,
                                              orientation, 
                                              intersects)

    rospy.loginfo(bearing)
        
    # управленияе взависимости от угла отклонения текущего
    # курса от курса на пересечение
    rot = k * bearing
    if abs(rot) > 1:
        rot = math.copysign(1, rot)

    send_command(0.7, rot)

    # отрисовка курса на пересечение для отладки
    intersect_marker.points = []    
    intersect_marker.points.append(array_to_point(position))
    intersect_marker.points.append(array_to_point(int_pos))    
    intersect_pub.publish(intersect_marker)    

    # отрисова окружности поиска для оладки
    publish_circle(position, radius)


#########################################################

if __name__ == '__main__':
    rospy.init_node('path_move')
    rospy.loginfo('path_move started')

    circle_pub = rospy.Publisher('circle', Marker, queue_size = 1)
    intersect_pub = rospy.Publisher('intersect', Marker, queue_size = 1)    
    commands_pub = rospy.Publisher('joy', Joy, queue_size = 100)

    rospy.Subscriber('/path', Path, path_callback)
    rospy.Subscriber('/zed/odom', Odometry, odom_callback)

    rospy.spin()
    rospy.loginfo('Stopping...')
    send_command(0, 0)