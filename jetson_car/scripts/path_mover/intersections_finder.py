#coding=utf-8
import rospy
import math
import numpy as np
import msg_helpers

# Нормирует вектор
def __norm(vec):
    return vec / np.linalg.norm(vec)


# проверяет, что точка на прямой лежит в пределах отрезка
# a - точка
# A, B - границы отрезка
def __check_bounds(p, A, B):
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
def __find_intersection(p1, p2, O, r, eps):
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
        if __check_bounds(A, p1, p2):
            return A
        else:
            return None
    
    if d > r:
        # Пересечений нет
        return None
    
    # иначе 2 пересечение
    
    # Расстояние от точки D до точек пересечения (A, B)
    # (одинаково в обе стороны)
    l = math.sqrt(r*r - d*d)
    
    # Отложим расстояние l от точки C вдоль  нормированного направляющего вектора
    # прямой p(px, py). Считаем, что точка p2 лежит дальше по траектории, чем
    # точка p1:
    p = __norm([px, py])
    C = [x, y]    
    A = C + l*p
    if __check_bounds(A, p1, p2):
        return A  

    return None

# Нахождение точек пересечения между окружностью и ломаной, заданной
# последовательностью точек
# path       - траектория (ломаная)
# O          - центр окружности
# r          - радиус окружности
# eps        - точность проверки 
# start      - с какого индекса надо начинать искать (в обе стороны)
# length     - какое количество точек искать, начиная со start
def find(path, O, r, eps, start, length):        
    #intersetcs = []
    #poses=path.poses[start:length]
    #print(start, length, len(poses))
    #for i in range(len(path.poses)-1):
    #    points = __find_intersections(msg_helpers.point_to_array(path.poses[i].pose.position),
    #                           msg_helpers.point_to_array(path.poses[i+1].pose.position),
    #                           O, r, eps)

    #    for p in points:
    #        intersetcs.append((i, p))
    
    #return intersetcs

    i = max(start - length/2, 0)
    intersect = None
    intersect_index = 0
    while ((i < start + length/2) or (intersect is None)) and (i < len(path.poses)-1):
        intersect_ = __find_intersection(msg_helpers.point_to_array(path.poses[i].pose.position),
                                           msg_helpers.point_to_array(path.poses[i+1].pose.position),
                                           O, r, eps)
        if intersect_ is not None:
            intersect_index = i
            intersect = intersect_

        i+=1

    return (intersect_index, intersect)
