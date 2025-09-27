#include <ros/ros.h>
#include <radio_hw/radio_remote.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "radio_remote_node");
  ros::NodeHandle nh;

  radio_hw::RadioRemote radio_remote;
  if (radio_remote.init(nh)) {
    ROS_INFO_STREAM("Radio remote initialized");
  } else {
    ROS_ERROR_STREAM("Failed to initialize Radio remote");
    return -1;
  }

  while(ros::ok())
  {
    radio_remote.read_data(ros::Time::now());
    ros::spinOnce();
  }

  return 0;
}