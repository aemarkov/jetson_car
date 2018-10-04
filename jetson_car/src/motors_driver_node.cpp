/*
	This node is a motors driver for jetson_car.
	It subscribes to raw control commands topic
	with pwm commands for left and right motors and
	send them in correct format to the STM32 via UART 
*/

#include <ros/ros.h>
#include <jetson_car/MotorsControl.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdint.h>


int8_t buffer[]={0x34, 0x27, 0x77, 0x10, 0x10};
int com_fd;

void callback(const jetson_car::MotorsControl msg)
{
	buffer[3] = msg.left;
	buffer[4] = msg.right;
	write(com_fd, buffer, sizeof(buffer));
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "motors_driver");
	ros::NodeHandle nh;
	ROS_INFO("Motors driver started");

	com_fd = open("/dev/ttyS0", O_RDWR);
	if(com_fd == -1)
	{
		ROS_ERROR("Unable to open UART");
		return -1;
	}

	auto sub = nh.subscribe("motors_commands", 100, callback);
	ros::spin();
	return 0;
}
