#!/bin/bash
# filepath: /home/keten/easy_ecat_master/src/rc_ecat_master/scripts/run_rc_ecat_node.sh

if [ "$EUID" -ne 0 ]; then
    exec sudo -E "$0" "$@"
fi


# 加载 ROS 系统环境
source /opt/ros/noetic/setup.bash

# 切换到工作空间根目录，并加载工作空间环境
cd /home/rc/RC_2026/py2_ws
source devel/setup.bash 

mon launch rc_ecat_master test_swerve.launch --name ethercat_master


