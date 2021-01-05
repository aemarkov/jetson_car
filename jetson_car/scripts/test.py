#!/usr/bin/env python
#coding=utf-8

import rospy
from jetson_car.msg import MotorsControl


if __name__ == '__main__':
    rospy.init_node('test')
    motors_control_pub = rospy.Publisher('motors_commands', MotorsControl, queue_size=100)

    cmd = MotorsControl()
    pwm = 60
    tmax = 3
    cmd.left = pwm
    cmd.right = pwm

    rate = rospy.Rate(10)
    t0 = rospy.get_time()
    while not rospy.is_shutdown():
        t = rospy.get_time() - t0
        print(t)
        motors_control_pub.publish(cmd)
        rate.sleep()
        if t >= tmax:
            break;