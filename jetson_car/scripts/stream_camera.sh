#!/bin/bash

PORT=5000
BAUD=4000000

while getopts ":hi:p:b:" opt
do
case $opt in
h)
    echo "./stream-camera.sh -i <ip> [-p <port> -b <baud>]"
    echo "Streams video from NVidia Jetson CSI-camera"
    echo "    -ip    client(!) ip address"
    echo "    -port  Streaming port, default $PORT"
    echo "    -baud  Baud rate in kb/s, default $BAUD"
    exit 0
    ;;
i)    IP=$OPTARG ;;
p)    PORT=$OPTARG ;;
b)    BAUD=$OPTARG ;;
*)
    echo "Invalid argument" ;;
esac
done

if [ -z $IP ]
then
    echo "Client IP address not set"
    echo "Use ./stream-camera -h to view help"
    exit 1
fi

echo "Starting streaming to $IP:$PORT with baudrate $BAUD"

gst-launch-1.0 nvcamerasrc fpsRange="30 30" intent=3 ! nvvidconv flip-method=0 \
! 'video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)I420, framerate=(fraction)30/1' ! \
omxh264enc control-rate=2 bitrate=$BAUD ! 'video/x-h264, stream-format=(string)byte-stream' ! \
h264parse ! rtph264pay mtu=1400 ! udpsink host=$IP port=$PORT sync=false async=false
