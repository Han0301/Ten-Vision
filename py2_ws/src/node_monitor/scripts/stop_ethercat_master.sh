#!/bin/bash

source /opt/ros/noetic/setup.bash

cd /home/rc/py_ws
source ./devel/setup.bash

rosrun node_monitor monitor_node ethercat_master rc_ecat_hw_node 2 "" 