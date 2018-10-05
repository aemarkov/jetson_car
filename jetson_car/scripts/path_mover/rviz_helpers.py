#coding=utf-8

import math
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import msg_helpers

class RvizHelpers:
    def __init__(self, circle_topic, intersect_topic):
        self.circle_pub = rospy.Publisher(circle_topic, Marker, queue_size = 1)
        self.intersect_pub = rospy.Publisher(intersect_topic, Marker, queue_size = 1)   
        
        self.circle_marker = self.__init_marker(Marker.LINE_STRIP, (0, 0, 1, 1))
        self.intersect_marker = self.__init_marker(Marker.LINE_LIST, (1, 1, 0, 1))
        

    def __init_marker(self, marker_type, rgba):
        marker = Marker()
        marker.type = marker_type
        marker.header.frame_id = 'map'
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]
        return marker

    # Публикует маркер кольца
    def circle(self, center, r):
        steps = 20
        self.circle_marker.points = []
        for i in range(steps+1):
            p =  Point()
            angle = 2 * math.pi / steps * i
            p.x = center[0] + math.cos(angle)*r
            p.y = center[1] + math.sin(angle)*r
            p.z = 0
            self.circle_marker.points.append(p)

        self.circle_pub.publish(self.circle_marker)

    # отрисовка курса на пересечение для отладки
    def intersection(self, position, intersections):
        self.intersect_marker.points = []
        for intersection in intersections:
            self.intersect_marker.points.append(msg_helpers.array_to_point(position))
            self.intersect_marker.points.append(msg_helpers.array_to_point(intersection))    
        self.intersect_pub.publish(self.intersect_marker)

