#!/bin/bash

if [ "$EUID" -ne 0 ]; then
    exec sudo -E "$0" "$@"
fi

# 加载 ROS 系统环境
source /opt/ros/noetic/setup.bash

# 切换到工作空间根目录，并加载工作空间环境
cd /home/rc/RC_2026/py2_ws
source devel/setup.bash 

# under controller_master,will rosrun nodes: radio_remote dribble_node pub_odom_node
#mon launch rc_ecat_master load_simple_swerve_controller.launch --name controller_master 
mon launch rc_ecat_master load_controller.launch --name controller_master
