#!/usr/bin/env python2

import os
import sys
import rospy
import time
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

DIR = '/home/nvidia/slogs'

ts = str(math.floor(time.time()))
ts_dot = ts.find('.')
if ts_dot != -1:
    # On some systems str on floored number produces result with a dot, on some don't
    ts = ts[:ts_dot]

T = None

def open_logs(ts, log_type, note=''):
    short_name = '%s_%s_%s.txt' % (ts, note, log_type)
    long_name  = '%s_%s_%s_full.txt' % (ts, note, log_type)
    fshort = open(os.path.join(DIR, short_name), 'w')
    flong  = open(os.path.join(DIR, long_name), 'w')
    fshort.write('X\tY\n')
    flong.write('t\tX\tY\tZ\tYaw\tPitch\tRoll\tQ.x\tQ.y\tQ.z\tQ.w\n')
    return fshort, flong


def do_log(f, ff, timestamp, pose):
    global T
    if T is None:
        T = timestamp

    # Short logging
    pos = pose.position
    f.write(str(pos.x) + '\t' + str(pos.y) + '\n')

    # Full logging
    dt = timestamp - T
    q = pose.orientation  # Quaternion
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([q.w, q.x, q.y, q.z])
    ff.write(str(dt) + '\t' + str(pos.x) + '\t' + str(pos.y) + '\t' + str(pos.z) + '\t' +
    str(yaw) + '\t' + str(pitch) + '\t' + str(roll) + '\t' +
    str(q.x) + '\t' + str(q.y) + '\t' + str(q.z) + '\t' + str(q.w) + '\n')

def log_zed(data):
    do_log(zed_f, zed_f_full, data.header.stamp, data.pose)
    try:
        pos, qrot = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        pose = Pose(Point(*pos), Quaternion(*qrot))
        do_log(lidar_f, lidar_f_full, data.header.stamp, pose)
    except(LookupException):
        pass


if __name__ == '__main__':
    rospy.init_node('simple_odologger')

    note = ''
    if len(sys.argv) > 1:
        note = sys.argv[1]

    zed_f, zed_f_full = open_logs(ts, 'zed', note)
    lidar_f, lidar_f_full = open_logs(ts, 'lidar', note)

    rospy.Subscriber('/zed/zed_node/pose', PoseStamped, log_zed)
    listener = tf.TransformListener()
    rospy.spin()
