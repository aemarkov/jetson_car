#coding=utf-8

import math
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import msg_helpers

class LinesHelper:
    def __init__(self, topic, color):
        self.pub = rospy.Publisher(topic, Marker, queue_size = 1)
        self.marker = self.__init_marker(color)

    def __init_marker(self, rgba):
        marker = Marker()
        marker.type = Marker.LINE_LIST
        marker.header.frame_id = 'map'
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]
        return marker

    def lines(self, lines):
        self.marker.points = []
        for line in lines:
            for i in range(len(line)-1):
                self.marker.points.append(msg_helpers.array_to_point(line[i]))
                self.marker.points.append(msg_helpers.array_to_point(line[i+1]))

        self.pub.publish(self.marker)
