// sensor_transformer.cpp
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

class SensorTransformer {
 private:
  ros::NodeHandle nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber action_sub_, radar_sub_;
  ros::Publisher action_pub_, radar_pub_;

 public:
  SensorTransformer() : tf_listener_(tf_buffer_) {
    // 订阅原始传感器数据（假设话题名为/action_sensor/pose和/radar_sensor/pose）
    action_sub_ = nh_.subscribe("/action1/pose", 10,
                                &SensorTransformer::actionCallback, this);
    radar_sub_ = nh_.subscribe("/radar_sensor/pose", 10,
                               &SensorTransformer::radarCallback, this);

    // 发布转换到base_link后的数据
    action_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/action_sensor/base_link_pose", 10);
    radar_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/radar_sensor/base_link_pose", 10);
  }

  void transformAndPublish(const geometry_msgs::PoseStamped& input,
                           ros::Publisher& pub) {
    geometry_msgs::PoseStamped output;
    try {
      // 关键转换操作：将数据转换到base_link坐标系
      tf_buffer_.transform(input, output, "base_link",
                           ros::Duration(0.1));  // 允许100ms时间容差
      pub.publish(output);
      ROS_INFO_THROTTLE(0.5, "pub the data!");
    } catch (tf2::TransformException& ex) {
      ROS_WARN("TF转换失败: %s", ex.what());
    }
  }

  void actionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    transformAndPublish(*msg, action_pub_);
  }

  void radarCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    transformAndPublish(*msg, radar_pub_);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "sensor_transformer");
  SensorTransformer transformer;
  ros::spin();
  return 0;
}