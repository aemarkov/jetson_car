#!/usr/bin/env python
#coding=utf-8

import sys
import time
import rospy
import math
from sensor_msgs.msg import Joy
from jetson_car.msg import MotorsControl

commands_pub = None
R = 1     #float(sys.argv[1])
r = 0.2   #float(sys.argv[2])
b = 0.25  # [0.08; 0.25]
run_time = 2
freq = 20
k_speed = 127
k_angle = 0
min_v = 0.08
omega = 0.17 * 1.08

def send_command(forward, rot):
    cmd = MotorsControl()
    cmd.left = forward - rot
    cmd.right = forward + rot

    if abs(cmd.left) > 127:
        cmd.left = math.copysign(127, cmd.left) 

    if abs(cmd.right) > 127:
        cmd.right = math.copysign(127, cmd.right) 

    print cmd.left, cmd.right
    motors_control_pub.publish(cmd)

# main
if __name__ == '__main__':
    rospy.init_node('circle_move')
    rospy.loginfo('circle_move started')

    motors_control_pub = rospy.Publisher('motors_commands', MotorsControl, queue_size=100)

    rate = rospy.Rate(freq)
    t0 = rospy.Time.from_sec(time.time()).to_sec()

    while True:
        #angle = r / (2 * math.pi * R)
        #speed = b
        
        #if abs(speed < min_v):
        #    speed = math.copysign(min_v, speed)

        dt = rospy.Time.from_sec(time.time()).to_sec() - t0
        angle = omega
        speed = b

        #speed = b
        #if dt < 1:
        #    angle = 0            
        #elif dt < 2:
        #    angle = omega            
        #elif dt < 3:
        #    angle = 0
        #else:
        #    break

        if dt > 12:
            break

        send_command(speed * k_speed, angle * k_speed)
        rate.sleep()

    cmd = MotorsControl()
    cmd.left = 0
    cmd.right = 0
    motors_control_pub.publish(cmd)    


