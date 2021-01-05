#!/bin/bash

PORT=5000

while getopts ":hp:" opt
do
case $opt in
h)
    echo "./view_stream.sh [-p <port>]"
    echo "View stream from NVIdia Jetson CSI-camera"
    echo "    -port Streaming port, default $PORT"
    exit 0
    ;;
p)    POPT=$OPTARG ;;
*)  echo "Invalid argument" ;;
esac
done

gst-launch-1.0 udpsrc port=$PORT ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! h264parse ! queue ! avdec_h264 ! autovideosink