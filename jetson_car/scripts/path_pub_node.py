#!/usr/bin/env python

# This node publish path from iFrund-compatible file

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

path_topic = 'local_path'
tf_frame   = 'map'

def get_pose(x,y):
	pose = PoseStamped()
	pose.pose.position.x = x
	pose.pose.position.y = y
	return pose


if __name__ == '__main__':
	rospy.init_node('path_pub_node')
	rospy.loginfo('path_pub_node started')

	if not rospy.has_param('~path_file'):
		rospy.logfatal('~path_file parameter not set')
		exit(0)

	path_file = rospy.get_param('~path_file')
	pub = rospy.Publisher(path_topic, Path, queue_size = 1, latch=True)

	path = Path()
	path.header.frame_id = tf_frame

	with open(path_file, 'r') as f:
		f.readline() # skip header
		for line in f:
			x, y = line.strip().split()
			path.poses.append(get_pose(float(x),float(y)))

	pub.publish(path)
	rospy.spin()
