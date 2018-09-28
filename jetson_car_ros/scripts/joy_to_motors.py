#!/usr/bin/env python

# This node convert joystick commands
# to motors pwm commands

import rospy
from jetson_car.msg import MotorsControl
from sensor_msgs.msg import Joy

MAX_PWM = None
motors_control_pub = None

def joy_cmd_callback(msg):
	forward = msg.axes[1]
	rot = msg.axes[2]

	cmd = MotorsControl()	
	cmd.left  =  forward * MAX_PWM
	cmd.right =  forward * MAX_PWM
	cmd.left  += rot * MAX_PWM
	cmd.right -= rot * MAX_PWM    

	motors_control_pub.publish(cmd)

if __name__ == '__main__':
	rospy.init_node('joy_to_motors')

	if not rospy.has_param('~max_pwm'):
		rospy.logfatal('Parameter max_pwm not set')
		exit(1)


	MAX_PWM = rospy.get_param('~max_pwm', 25)
	
	rospy.loginfo("MAX_PWM:   %d", MAX_PWM)

	rospy.Subscriber('joy', Joy, joy_cmd_callback)
	motors_control_pub = rospy.Publisher('motors_commands', MotorsControl, queue_size=100)

	rospy.loginfo('joy_to_msg started')
	rospy.spin()
