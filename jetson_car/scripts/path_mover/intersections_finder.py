#coding=utf-8

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

# угол между вектором и осью x
def __vector_angle(vec):
    return math.atan2(vec[1], vec[0])

# Нахождение точек пересечения между окружностью и отрезком
# p1,p2  - границы отрезка
# O      - центр окружности
# r      - радиус окружности
# eps    - точность проверки 
def __find_intersections(p1, p2, O, r, eps):
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
    p = __norm([px, py])
    C = [x, y]
    A = C - l*p
    B = C + l*p    
        
    # Проверка границ отрезка
    points = []
    if __check_bounds(A, p1, p2):
        points.append(A)
        
    if __check_bounds(B, p1, p2):
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
        p = __find_intersections(msg_helpers.point_to_array(path.poses[i].pose.position),
                               msg_helpers.point_to_array(path.poses[i+1].pose.position),
                               O, r, eps)
        intersetcs.extend(p)
    
    return intersetcs

# находит пересечение, угол на который меньше всего отличатеся от текущего курса
def find_closest_intersect(pos, orientation, intersects):    
    return  min(map(lambda x: (x, __vector_angle(x - pos) - orientation), intersects), key=lambda x: abs(x[1]))
