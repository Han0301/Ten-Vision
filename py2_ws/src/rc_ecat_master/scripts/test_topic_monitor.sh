#!/bin/bash

# 加载ROS环境
source /opt/ros/noetic/setup.bash

# 加载工作空间环境
source /home/rc/py_ws/devel/setup.bash

# 执行监控脚本
python3 /home/rc/py_ws/src/rc_ecat_master/scripts/ros_topic_monitor.py