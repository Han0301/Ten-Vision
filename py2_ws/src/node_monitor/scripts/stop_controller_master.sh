#!/bin/bash

source /opt/ros/noetic/setup.bash

cd /home/rc/py_ws
source ./devel/setup.bash

rosrun node_monitor monitor_node controller_master controller_spawner 2 "" 