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
#include <termios.h>
#include <errno.h>

#define SERIAL "/dev/ttyTHS2"


int8_t buffer[]={0x34, 0x27, 0x77, 0x00, 0x00};
int serial_fd;

void callback(const jetson_car::MotorsControl msg)
{
    buffer[3] = msg.left;
    buffer[4] = msg.right;
    write(serial_fd, buffer, sizeof(buffer));
}

int open_and_configure_serial(const char* device)
{
    struct termios tty;

    int fd = open(device, O_RDWR);
    if(fd == -1) {
        ROS_ERROR_STREAM("Unable to open UART: " << strerror(errno));
        goto err;
    }

    if (tcgetattr(fd, &tty) != 0) {
        ROS_ERROR_STREAM("Unable to read TTY settings: " << strerror(errno));
        goto err;
    }

    // Baud 115200, 8N1, no parity, no HW control
    tty.c_cflag &= ~PARENB;   // no parity
    tty.c_cflag &= ~CSTOPB;   // 1 stop bit
    tty.c_cflag &= ~CSIZE;    // clear number of bits
    tty.c_cflag |= CS8;       // set 8 bits
    tty.c_cflag &= ~CRTSCTS;  // no HW control
    cfsetispeed(&tty, B115200);
    // Well, there are more settings (line discipline, echo etc), but I think, this is enough.
    // Actually, I think baud is only one we need...

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ROS_ERROR_STREAM("Unable to write TTY settings: " << strerror(errno));
        goto err;
    }

    return fd;
err:
    if (fd != -1) {
        close(fd);
    }
    return -1;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motors_driver_node");
    ros::NodeHandle nh;
    ROS_INFO("Motors driver started");

    if ((serial_fd = open_and_configure_serial(SERIAL)) == -1) {
        ROS_ERROR_STREAM("Unable to open UART");
        return -1;
    }

    auto sub = nh.subscribe("motors_commands", 100, callback);
    ros::spin();
    return 0;
}
