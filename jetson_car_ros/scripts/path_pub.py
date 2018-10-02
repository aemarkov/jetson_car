#!/usr/bin/env python

# This node publish example path

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

def get_pose(x,y):
	pose = PoseStamped()
	#pose.header = Header()
	pose.pose.position.x = x
	pose.pose.position.y = y
	return pose


if __name__ == '__main__':
	rospy.init_node('path_pub')
	rospy.loginfo('path_pub started')
	pub = rospy.Publisher('path', Path, queue_size = 1, latch=True)
	path = Path()
	# 	path.header = Header()
	path.header.frame_id = 'map'
	path.poses = []	

	x = 0
	y = 0

	#for _ in range(15):		
	#	x+=0.1
	#	path.poses.append(get_pose(x,y))

	#for _ in range(15):
	#	pose = PoseStamped()
	#	y+=0.1
	#	path.poses.append(get_pose(x,y))

	#path.poses.append(get_pose(0,0))
	#path.poses.append(get_pose(1.3,0))
	#path.poses.append(get_pose(1.3,-1.1))
	#path.poses.append(get_pose(4.5,-1.1))
	#path.poses.append(get_pose(4.5,-3))
	#path.poses.append(get_pose(3.5,-3))

	path.poses.append(get_pose(0, 0))
	path.poses.append(get_pose(1, 0))
	path.poses.append(get_pose(1, -1))
	path.poses.append(get_pose(2, -1))
	path.poses.append(get_pose(1.5, 0))
	path.poses.append(get_pose(1.5, 1))

	pub.publish(path)
	rospy.spin()