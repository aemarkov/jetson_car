#!/usr/bin/env python
#coding=utf-8

# This node convert joystick commands
# to motors pwm commands

import rospy
from jetson_car.msg import MotorsControl
from sensor_msgs.msg import Joy

MAX_PWM = None
motors_control_pub = None

def trim(pwm):
	if pwm > MAX_PWM:
		return MAX_PWM
	if pwm < -MAX_PWM:
		return -MAX_PWM
	return pwm

def joy_cmd_callback(msg):
	forward = msg.axes[1]
	rot = msg.axes[2]

	left  =  forward * MAX_PWM
	right =  forward * MAX_PWM
	left  += rot * MAX_PWM
	right -= rot * MAX_PWM

	cmd = MotorsControl()
	cmd.left = trim(left)
	cmd.right = trim(right)

	motors_control_pub.publish(cmd)

if __name__ == '__main__':
	rospy.init_node('joy_to_motors')
	
	MAX_PWM = rospy.get_param('max_pwm', 15)	
	rospy.loginfo("MAX_PWM:   %d", MAX_PWM)

	rospy.Subscriber('joy', Joy, joy_cmd_callback)
	motors_control_pub = rospy.Publisher('motors_commands', MotorsControl, queue_size=100)
	
	rospy.spin()