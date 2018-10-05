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
# start  - с какого индекса надо начинать искать
# length - какое количество точек искать, начиная со start
def find_path_intersections(path, O, r, eps, start = 0, length = -1):        
    intersetcs = []
    poses=path.poses[start:length]
    print(start, length, len(poses))
    for i in range(len(poses)-1):
        points = __find_intersections(msg_helpers.point_to_array(path.poses[i].pose.position),
                               msg_helpers.point_to_array(path.poses[i+1].pose.position),
                               O, r, eps)

        for p in points:
            intersetcs.append((i, p))
    
    return intersetcs


def __rot_matrix(angle):
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[c, -s],
                     [s,  c]])

def __rot_vector(vec, angle):
    return np.matmul(__rot_matrix(angle), vec)

def __vec_of_angle(angle):
    return __rot_vector(np.array([1,0]), angle)

# находит пересечение, угол на который меньше всего отличатеся от текущего курса
def find_closest_intersect(pos, orientation, intersects):  
    
    # Берем пересечение с самым большим
    intersect = max(intersects, key=lambda x: x[0])
    vec_to_intersect =  intersect[1] - pos
    orientation_vec = __vec_of_angle(orientation)
    bearing = math.atan2(np.cross(orientation_vec, vec_to_intersect), np.dot(vec_to_intersect, orientation_vec))
    return (intersect[0], intersect[1], bearing)
    #return  max(map(lambda x: (x[1], __vector_angle(x[1] - pos) - orientation, x[0]), intersects), key=lambda x: x[2])
