#!/usr/bin/env python
#coding=utf-8

# This node convert joystick commands
# to motors pwm commands

import rospy
from jetson_car.msg import MotorsControl
from sensor_msgs.msg import Joy

WTF_MAX_PWM = 127    # Absoutly max value
MAX_PWM = None       # user-set value (< WTF_MAX_PWM)
IS_LOOP = None

motors_control_pub = None
global_cmd = MotorsControl()

def trim(pwm):
    if pwm > WTF_MAX_PWM:
        return WTF_MAX_PWM
    if pwm < -WTF_MAX_PWM:
        return -WTF_MAX_PWM
    return pwm

def create_cmd(msg):
    forward = msg.axes[1]
    rot = msg.axes[2]

    left  =  forward * MAX_PWM
    right =  forward * MAX_PWM
    left  += rot * MAX_PWM 
    right -= rot * MAX_PWM

    cmd = MotorsControl()
    cmd.left = trim(left)
    cmd.right = trim(right)
    return cmd

def joy_cmd_callback(msg):
    global global_cmd
    if IS_LOOP:
        global_cmd = create_cmd(msg)
    else:
        motors_control_pub.publish(create_cmd(msg))

if __name__ == '__main__':
    rospy.init_node('joy_to_motors')
    
    MAX_PWM = rospy.get_param('~max_pwm', 15)    
    IS_LOOP = rospy.get_param('~is_loop', False)
    rospy.loginfo("MAX_PWM:   %d", MAX_PWM)
    rospy.loginfo("IS_LOOP:   %d", IS_LOOP)

    rospy.Subscriber('joy', Joy, joy_cmd_callback)
    motors_control_pub = rospy.Publisher('motors_commands', MotorsControl, queue_size=100)
    
    if IS_LOOP:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            motors_control_pub.publish(global_cmd)            
            rate.sleep()
    else:
        rospy.spin()