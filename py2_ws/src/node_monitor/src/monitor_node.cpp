/**
 * @file monitor_node.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-07-31
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note : 通过mon launch 直接操作节点监控，实现暂时关闭和完全重启的快捷键
 *         实现效果即：全部放入脚本启动器之后可以无需关联其他位置的数据
 * @versioninfo :
 */
#include <ros/ros.h>
#include <rosmon_msgs/StartStop.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "monitor_node");
  ros::NodeHandle nh;

  if (argc < 5) {
    ROS_ERROR_STREAM("Usage: Please input 4 parameters!");
    ROS_ERROR_STREAM(
        "Usage: arg-1: [rosmon-ns] \r\n arg-2: [node-name] \r\n arg-3: "
        "[node-action] \r\n arg-4: [node-ns]");
    return -1;
  }

  std::string rosmon_ns = argv[1];
  std::string node_name = argv[2];
  int action = std::atoi(argv[3]);
  std::string node_ns = argv[4];

  std::string srv_name = rosmon_ns + "/start_stop";
  ROS_INFO_STREAM("[rosservice]: rosmon is called!" << srv_name.c_str());

  // 等待服务
  ros::service::waitForService(srv_name);
  ros::ServiceClient client =
      nh.serviceClient<rosmon_msgs::StartStop>(srv_name);

  rosmon_msgs::StartStop srv;
  srv.request.node = node_name;
  srv.request.ns = node_ns;
  srv.request.action = action;

  if (client.call(srv)) {
    ROS_INFO_STREAM("called successfully! node: "
                    << node_ns + '/' + node_name << "is called to " << action
                    << "\r\n"
                    << "[1] to START; [2] to STOP; [3] to RESTART");
  } else {
    ROS_ERROR_STREAM("called failed!");
    return 1;
  }

  return 0;
}