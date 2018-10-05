#!/usr/bin/env python
#coding=utf-8

# This node convert joystick commands
# to motors pwm commands

import rospy
from jetson_car.msg import MotorsControl
from sensor_msgs.msg import Joy

MAX_PWM = None
motors_control_pub = None
stop_cnt = 10

def joy_cmd_callback(msg):
	global stop_cnt
	forward = msg.axes[1]
	rot = msg.axes[2]

	cmd = MotorsControl()	
	cmd.left  =  forward * MAX_PWM
	cmd.right =  forward * MAX_PWM
	cmd.left  += rot * MAX_PWM
	cmd.right -= rot * MAX_PWM    

	stop_cnt = 10
	motors_control_pub.publish(cmd)

if __name__ == '__main__':
	rospy.init_node('joy_to_motors')
	
	MAX_PWM = rospy.get_param('max_pwm', 25)	
	rospy.loginfo("MAX_PWM:   %d", MAX_PWM)

	rospy.Subscriber('joy', Joy, joy_cmd_callback)
	motors_control_pub = rospy.Publisher('motors_commands', MotorsControl, queue_size=100)
	
	#rospy.spin()
	r = rospy.Rate(10)
	# Отправляем команду остановки, если команды долго не поступают
	while not rospy.is_shutdown():
		stop_cnt-=1
		if stop_cnt < 0:
			stop_cnt=0
			msg = MotorsControl()
			msg.left = 0
			msg.right = 0
			motors_control_pub.publish()
		r.sleep()

