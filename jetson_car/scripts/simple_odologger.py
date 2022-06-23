#!/usr/bin/env python

import rospy
import time
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

ts = str(math.floor(time.time()))
ts_dot = ts.find('.')
if ts_dot != -1:
	# On some systems str on floored number produces result with a dot, on some don't
	ts = ts[:ts_dot]
lidar_f = open('/home/nvidia/slogs/' + ts + '_lidar.txt', 'w')
lidar_f_full = open('/home/nvidia/slogs/' + ts + '_lidar_full.txt', 'w')
zed_f = open('/home/nvidia/slogs/' + ts + '_zed.txt', 'w')
zed_f_full = open('/home/nvidia/slogs/' + ts + '_zed_full.txt', 'w')
lidar_f.write('X\tY\n')
lidar_f_full.write('t\tX\tY\tZ\tYaw\tPitch\tRoll\tQ.x\tQ.y\tQ.z\tQ.w\n')
zed_f.write('X\tY\n')
zed_f_full.write('t\tX\tY\tZ\tYaw\tPitch\tRoll\tQ.x\tQ.y\tQ.z\tQ.w\n')
T = None

def do_log(f, ff, pose):
	global T
	if T is None:
		T = rospy.get_time()
	pos = pose.position
	# Short logging
	f.write(str(pos.x) + '\t' + str(pos.y) + '\n')
	# Flush ?
	#f.flush()
	# Full logging
	dt = rospy.get_time() - T
	q = pose.orientation  # Quaternion
	roll, pitch, yaw = tf.transformations.euler_from_quaternion([q.w, q.x, q.y, q.z])
	ff.write(str(dt) + '\t' + str(pos.x) + '\t' + str(pos.y) + '\t' + str(pos.z) + '\t' +
		str(yaw) + '\t' + str(pitch) + '\t' + str(roll) + '\t' +
		str(q.x) + '\t' + str(q.y) + '\t' + str(q.z) + '\t' + str(q.w) + '\n')

def log_lidar(data):
	do_log(lidar_f, lidar_f_full, data.pose.pose)

def log_zed(data):
	do_log(zed_f, zed_f_full, data.pose)

def listener():
	global T
	rospy.init_node('simple_odologger')
	rospy.Subscriber('/integrated_to_init', Odometry, log_lidar)
	rospy.Subscriber('/zed/zed_node/pose', PoseStamped, log_zed)
	rospy.spin()

if __name__ == '__main__':
	listener()
