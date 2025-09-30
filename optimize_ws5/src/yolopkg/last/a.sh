#!/bin/bash
export MVCAM_SDK_PATH=/opt/MVS

export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/32:$LD_LIBRARY_PATH

source /opt/ros/noetic/setup.bash 
sleep 3 
source /home/robocon/RC/project6_ws/devel/setup.bash
sleep 3 
roslaunch yolopkg yolo.launch
