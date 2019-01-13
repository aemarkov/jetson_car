#!/usr/bin/env python
#coding=utf-8

# Allow to draw trajectory manually in RViz

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
import os
from datetime import datetime

path_topic = 'path'
tf_frame = 'map'

def create_marker():
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3
    marker.color.r = 0.0
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    return marker

def create_interactive_marker():
    # Interactiove marker
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = tf_frame
    int_marker.name = "drawer_marker"
    int_marker.description = "Drawer marker"

    # Interactive marker control
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.orientation_mode = InteractiveMarkerControl.VIEW_FACING
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    control.independent_marker_orientation = True    
    control.markers.append(create_marker())
    int_marker.controls.append(control)
    return int_marker

def get_filename():
    time = datetime.now()
    filename = 'path_'+time.strftime("%Y.%m.%d_%H.%M.%S")+".txt"
    return filename

def marker_cb(feedback):
    p = feedback.pose.position    
    pose = PoseStamped()
    pose.pose.position = p
    path.poses.append(pose)
    pub.publish(path)

    if path_file!=None:
        path_file.write(str(p.x)+'\t'+str(p.y)+'\n')

path_directory = None
path_file = None

if __name__=="__main__":
    rospy.init_node("path_drawer_node")
    rospy.loginfo('path_drawer_node started')

    if rospy.has_param('~path_directory'):
        path_directory = rospy.get_param('path_directory')
        if not os.path.exists(path_directory):
            os.makedirs(path_directory)
        
        filename = os.path.join(path_directory, get_filename())
        path_file = open(filename, 'w')
        path_file.write('X\tY\n')
        rospy.loginfo('Path is logged to: %s', filename)


    # Create topic
    pub = rospy.Publisher(path_topic, Path, queue_size = 1, latch=True)
    path = Path()
    path.header.frame_id = tf_frame


    # Create marker
    server = InteractiveMarkerServer("drawer_marker")
    draw_marker = create_interactive_marker()
    server.insert(draw_marker, marker_cb)
    server.applyChanges()
    rospy.spin()