#!/bin/bash

source /opt/ros/noetic/setup.bash

cd /home/rc/py_ws
source ./devel/setup.bash

roslaunch node_monitor restart_all_master.launch