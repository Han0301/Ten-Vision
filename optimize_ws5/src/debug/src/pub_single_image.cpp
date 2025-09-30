#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

int main(int argc, char**argv)
{
    // 初始化ROS节点
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "pub_single_image_node");
    ros::NodeHandle nh;
    
    // 创建图像传输对象，用于发布图像
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/color/image_raw", 1);
    
    // 图片路径，可以通过ROS参数设置，默认为当前目录下的test.jpg
    std::string image_path;
    nh.param<std::string>("image_path", image_path, "/home/maple/develop/optimize_ws4/src/debug/image/image6.png");
    
    // 使用OpenCV读取图片
    cv::Mat image = cv::imread(image_path);
    if (image.empty()) {
        ROS_ERROR_STREAM("无法读取图片，请检查路径: " << image_path);
        return -1;
    }
    
    // 创建cv_bridge对象，用于转换OpenCV图像到ROS消息


    // 发布图像消息
    while(ros::ok())
    {
        cv_bridge::CvImage cv_image;
        cv_image.header.stamp = ros::Time::now();  // 设置时间戳
        cv_image.header.frame_id = "camera_frame"; // 设置坐标系
        cv_image.encoding = sensor_msgs::image_encodings::BGR8; // 设置编码格式
        cv_image.image = image;                    // 赋值图像数据
        pub.publish(cv_image.toImageMsg());
        ROS_INFO_STREAM("成功发布图片: " << image_path);
        // 短暂休眠确保消息被发布
        //ros::Duration(0.1).sleep();
        //ros::spinOnce();
        cv::imshow("image",image);
        cv::waitKey(10);
    }

    cv::destroyAllWindows();
    return 0;
}
