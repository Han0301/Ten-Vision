#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cstdbool>
#include <iostream>

struct param
{
    int _number;
    std::string _name_raw;
    std::string _name_compress;
};

class pub
{
public:
    pub(int number, std::string name_raw, std::string name_compress):_nh("/"),_it(_nh), _cap(number,cv::CAP_V4L2), 
    _raw_pub(_it.advertise(name_raw, 10)),
    _compressed_pub(_nh.advertise<sensor_msgs::CompressedImage>(name_compress, 10)),
    _cap_open(false)
    {
        if (_cap.isOpened())
        {
            _cap_open = true;
        }
    } 
    pub():_nh("/"),_it(_nh)
    {
        get_param();
        _raw_pub = _it.advertise(p._name_raw, 10);
        _compressed_pub = _nh.advertise<sensor_msgs::CompressedImage>(p._name_compress, 10);
        _cap.open(p._number,cv::CAP_V4L2);
        if (_cap.isOpened())
        {
            std::cout<<"_cap.isOpened()"<<std::endl;
            _cap_open = true;
        }
        else
        {
            while(!_cap.isOpened())
            {
                _cap.open(p._number,cv::CAP_V4L2);
            }
        }
    }
    void pub_image();//发布原图
    void pub_compress_img();//发布压缩图片
    void get_param();//获得参数
    ~pub()
    {
        if(_cap_open)
        {
            _cap.release();
        }
        _raw_pub.shutdown();
        _compressed_pub.shutdown();
    }
private:
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;
    cv::VideoCapture _cap;
    image_transport::Publisher _raw_pub;
    ros::Publisher _compressed_pub;
    bool _cap_open = false;
    param p;
};




void pub::pub_image()
{
    if(_cap_open)
    {
        cv::Mat frame;
        _cap >> frame;
        if (frame.empty()) {
            ROS_WARN("frame are empty");
            return;
        }
        sensor_msgs::ImagePtr raw_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        raw_msg->header.stamp = ros::Time::now();
        _raw_pub.publish(raw_msg);
    }
    else
    {
        ROS_WARN("Can not open video");
    }
}
void pub::pub_compress_img()
{
    if(_cap_open)
    {
        cv::Mat frame;
        _cap >> frame;
        if (frame.empty()) {
            ROS_WARN("frame are empty");
            return;
        }
        sensor_msgs::CompressedImage compressed_msg;
        compressed_msg.header.stamp = ros::Time::now();
        compressed_msg.format = "jpeg";
        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params.push_back(80);  // 质量参数 (0-100)
        if (cv::imencode(".jpg", frame, compressed_msg.data, compression_params)) {
            _compressed_pub.publish(compressed_msg);
        } else {
            ROS_ERROR("图像压缩失败");
        }
    }
    else
    {
        ROS_WARN("Can not open video");
    }
}

void pub::get_param()
{
    _nh.param("number",p._number,2);
    _nh.param("image_name", p._name_raw,std::string("camera/iamge_raw"));
    _nh.param("compress_image_name", p._name_compress,std::string( "camera/compress_image"));
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "usb_cam_node");
    pub publish;
    //发布频率
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::Time tic = ros::Time::now();
        publish.pub_image();
        //publish.pub_compress_img();
        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO("publish.pub_image(): %f", (ros::Time::now() - tic).toSec());
    }
    return 0;
    
}


