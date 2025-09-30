#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include "rc_msgs/serial.h"
#include "rc_msgs/yolo_detectorArray.h"
#include "rc_msgs/yolo_detector.h"
#include <iostream>
#include <stdbool.h>
#include <chrono>  // 核心计时库
#define MAX_DATA_LENGTH 5 // 最大数据长度(浮点数个数)



//-------------------------------------------

#include <unordered_map>
#include <vector>

#include <string>
// 直接测试命令： g++ -std=c++11 -o split_image  split_image.cpp  `pkg-config --cflags --libs opencv4`  
//              chmod +x split_image
//              ./ split_image
/**************************
 * ROI组枚举：定义不同的ROI分组（可扩展）
 *************************/
enum class ROI_Group {
    GROUP_start,  
    GROUP_middle  
};

/**************************
 * 核心结构体：存储分割结果及元信息
 *************************/
struct SplitImage {
    cv::Mat image;          // 分割并resize后的图像（640x640）
    int place;              // ROI在组内的序号（用户指定，可跳跃）
    int class_label;        // 类别标签（暂时保留）

    // 默认构造函数（兼容vector）
    SplitImage() = default;

    // 带参构造函数（安全初始化）
    SplitImage(const cv::Mat& resized_img, int p, int c)
        : image(resized_img.clone()),  // 深拷贝图像
          place(p),
          class_label(c) {}
};

/**************************
 * 图像分割管理类：支持按组管理ROI、按指定组分割
 * 修改说明：
 *   - ROI信息在类初始化时已设置好
 *   - 删除了所有添加ROI的公开接口
 *************************/
class GroupedROISplitter {
public:
    // GroupedROISplitter() {
    //     // 初始化GROUP_start组的ROI
    //     group_rois_[ROI_Group::GROUP_start] = {
    //         {2, cv::Rect(271, 388, 205, 178)},
    //         {3, cv::Rect(680, 388, 226, 183)},
    //         {4, cv::Rect(61, 323, 195, 138)},
    //         {5, cv::Rect(425, 325, 170, 125)},
    //         {7, cv::Rect(240, 280, 140, 105)},
    //         {10, cv::Rect(346, 253, 115, 78)}
    //     };

    //     // 初始化GROUP_middle组的ROI
    //     group_rois_[ROI_Group::GROUP_middle] = {
    //         {4, cv::Rect(1, 725, 299, 351)},
    //         {5, cv::Rect(750, 756, 500, 322)},
    //         {6, cv::Rect(1596, 650, 324, 370)},
    //         {7, cv::Rect(205, 510, 376, 216)},
    //         {8, cv::Rect(855, 488, 316, 205)},
    //         {9, cv::Rect(1280, 486, 445, 230)},
    //         {10, cv::Rect(445, 306, 265, 167)},
    //         {11, cv::Rect(881, 363, 247, 118)},
    //         {12, cv::Rect(1216, 283, 277, 195)}
    //     };
    // }
    GroupedROISplitter() {
        // 初始化GROUP_start组的ROI
        group_rois_[ROI_Group::GROUP_start] = {
            {2, cv::Rect(241, 388, 225, 178)},
            {3, cv::Rect(680, 388, 226, 183)},
            {4, cv::Rect(0, 323, 195, 138)},
            {5, cv::Rect(395, 325, 170, 125)},
            {7, cv::Rect(140, 280, 140, 85)},
            {10, cv::Rect(266, 203, 155, 108)}
        };

        // 初始化GROUP_middle组的ROI
        group_rois_[ROI_Group::GROUP_middle] = {
            {4, cv::Rect(1, 725, 299, 351)},
            {5, cv::Rect(750, 756, 500, 322)},
            {6, cv::Rect(1596, 650, 324, 370)},
            {7, cv::Rect(205, 510, 376, 216)},
            {8, cv::Rect(855, 488, 316, 205)},
            {9, cv::Rect(1280, 486, 445, 230)},
            {10, cv::Rect(445, 306, 265, 167)},
            {11, cv::Rect(881, 363, 247, 118)},
            {12, cv::Rect(1216, 283, 277, 195)}
        };
    }
    // 处理输入图像：自动清空旧结果 → 按指定组的ROI分割 → 存入容器
    void processImage(const cv::Mat& input, ROI_Group target_group);
    
    // 获取当前处理结果（对应某组ROI的分割）
    const std::vector<SplitImage>& getFrames() const;
    
    // 手动清空结果容器
    void clearFrames();

//private:
    //内部实现：按指定组的ROI分割图像
    std::vector<SplitImage> splitWithGroupROIs(const cv::Mat& input, ROI_Group target_group);

//private:
    std::vector<SplitImage> frames_;  // 存储当前处理结果（某组ROI的分割）
    // 按组存储ROI：外层键=组类型，内层键=组内ROI序号，值=ROI区域
    // ROI信息在构造函数中已初始化
    std::unordered_map<ROI_Group, std::unordered_map<int, cv::Rect>> group_rois_;
};

void GroupedROISplitter::processImage(const cv::Mat& input, ROI_Group target_group) {
    frames_.clear();  // 关键：新输入时清空旧结果，确保容器仅存当前组的分割
    if (input.empty()) {
        std::cerr << "[Error] 输入图像为空！";
        return;
    }

    // 检查目标组是否有可用ROI
    auto group_it = group_rois_.find(target_group);
    if (group_it == group_rois_.end() || group_it->second.empty()) {
        std::cerr << "[Warning] 目标组(" << static_cast<int>(target_group) << ")未配置ROI,请先添加";
        return;
    }

    // 执行分组分割
    auto split_result = splitWithGroupROIs(input, target_group);
    
    // 移动语义优化性能（避免拷贝）
    frames_.reserve(split_result.size());
    for (auto& item : split_result) {
        frames_.push_back(std::move(item));         // 将分割结果移动到容器中
    }
}

const std::vector<SplitImage>& GroupedROISplitter::getFrames() const {
    return frames_;
}

void GroupedROISplitter::clearFrames() {
    frames_.clear();
}

std::vector<SplitImage> GroupedROISplitter::splitWithGroupROIs(const cv::Mat& input, ROI_Group target_group) {
    std::vector<SplitImage> result;
    auto group_it = group_rois_.find(target_group);
    if (group_it == group_rois_.end() || group_it->second.empty()) {
        return result;
    }

    const auto& group_rois = group_it->second;  // 目标组的ROI映射（序号→区域）
    cv::Rect image_boundary(0, 0, input.cols, input.rows);  // 图像边界（用于ROI越界检查）

    // 遍历目标组的所有ROI
    for (const auto& roi_pair : group_rois) {
        int place_id = roi_pair.first;                  // ROI在组内的序号
        const cv::Rect& roi_region = roi_pair.second;   // ROI区域

        // 1. ROI越界检查：确保ROI在图像范围内
        cv::Rect valid_roi = roi_region & image_boundary;
        if (valid_roi.empty()) {
            std::cerr << "[Warning] 组(" << static_cast<int>(target_group)
                      << ")的ROI序号(" << place_id << ")越界，跳过！";
            continue;
        }

        // 2. 提取ROI区域
        cv::Mat roi_block = input(valid_roi);

        // 3. Resize到固定尺寸（640x640）
        cv::Mat resized_block;
        cv::resize(roi_block, resized_block, cv::Size(640, 640));

        // 4. 构造结果结构体
        result.emplace_back(resized_block, place_id, 0);  // class_label暂设为0
    }

    return result;
}


//-------------------------------------------



union data // 数据域联合体(允许以不同方式访问相同内存)
{
    float msg_get[MAX_DATA_LENGTH];
    uint8_t buff_msg[MAX_DATA_LENGTH * 4];
};


// struct SplitImage
// {
//     cv::Mat image;
//     int place;
//     int class_label;
// };

// class CustomROISplitter
// {
// public:
//     std::vector<SplitImage> frames_;

// };

class map_sever
{
public:
    map_sever(): _nh(), _it(_nh)
    {
        _image_sub = _it.subscribe("/camera/color/image_raw", 2, &map_sever::imageCallback, this);
        _serial_sub = _nh.subscribe("/serial/status", 1, &map_sever::serialCallback, this);
        //_bbox_sub = _nh.subscribe("/yolo/detections", 10, &map_sever::yolodetector, this);
        _pub_image = _it.advertise("/camera/map_server", 2);
        _pub_map = _nh.advertise<rc_msgs::serial>("/serial/map", 10);
        for(int i = 0; i < MAX_DATA_LENGTH * 4; i++)
        {
            _data.buff_msg[i] = 0;
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            if(_enable_image)
            {
                // 将ROS图像消息转换为OpenCV格式（BGR8编码）
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                _image =  cv_ptr->image;
                _enable_image = false;
                process();
                map_send();
                _flag = 0;
            }

        }
        catch (cv_bridge::Exception& e)
        {
            // 处理转换异常 
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void serialCallback(const rc_msgs::serialConstPtr& msg)
    {
        if(_flag == 0)
        {
            _flag = msg->data[0];
            if(_flag == 1)//开始标定
            {
                _enable_image = true;
            }
        }
    }
    
    void yolodetector(const rc_msgs::yolo_detectorArray::ConstPtr msg)
    {
        if(!_enable_yolo)
        {
            for (size_t i = 0; i < msg->boxes.size(); ++i) 
            {
                const auto& bbox = msg->boxes[i];
                _enable_yolo = true;
                _classes = bbox.cls;
                return;
            }
        }
    } 

    void process()
    {
        //发图片给han处理
        //----------------------------------------------------------
        ROI_Group current_group;        // 用一个变量控制分组选择
        current_group = ROI_Group::GROUP_start;
        //current_group = ROI_Group::GROUP_middle;
        // 3. 处理ROI
        _han.clearFrames(); 
        _han.processImage(_image, current_group);
        //std::cout << "[INFO] GROUP_start 分割结果数量：" << _han.getFrames().size() << std::endl;
        std::cout<<"分割结果数量"<<_han.frames_.size()<<std::endl;

        //------------------------------------------------------------
        int length = _han.frames_.size();
        for(int i = 0; i < length; i++)
        {
            ros::Rate sl(10);
            _bbox_sub = _nh.subscribe("/yolo/detections", 10, &map_sever::yolodetector, this);
            sl.sleep();
            cv::Mat image = _han.frames_[i].image;
            // 创建ROS图像消息
            sensor_msgs::ImagePtr msg;
            // 将OpenCV图像转换为ROS消息
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            _pub_image.publish(msg);
            auto start = std::chrono::steady_clock::now();
            ros::Rate sleep(1000);
            while(!_enable_yolo)
            {
                sleep.sleep();
                auto end = std::chrono::steady_clock::now();
                auto duration = end - start;
                double seconds_double = static_cast<double>(duration.count()) / 1e9;
                if(seconds_double > 0.2)
                {
                    _enable_yolo = true;
                    _classes = 4;
                }
                ros::spinOnce();
            }
            _han.frames_[i].class_label = _classes;
            _classes = 0;
            _enable_yolo = false;
            std::cout<<i<<std::endl;
        }

    }

    void map_send()
    {
        int length = _han.frames_.size();
        for(int i = 0; i < length; i++)
        {
            _data.buff_msg[2*i] = _han.frames_[i].place;
            _data.buff_msg[2*i+1] = _han.frames_[i].class_label;
            std::cout<<"_han.frames_[i].place"<<_han.frames_[i].place<<std::endl;
            std::cout<<"_han.frames_[i].class_label"<<_han.frames_[i].class_label<<std::endl;
        }
        rc_msgs::serial serial_pub;
        serial_pub.stamp = ros::Time::now();
        for(int i = 0; i < length / 2 + 1; i++)
        {
            serial_pub.data.push_back(_data.msg_get[i]);
        }
        _pub_map.publish(serial_pub);
        _pub_map.publish(serial_pub);
        _pub_map.publish(serial_pub);
        std::cout<<"_pub_map.publish(serial_pub)"<<std::endl;
    }


private:
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;// 创建图像传输对象
    image_transport::Subscriber _image_sub;
    image_transport::Publisher _pub_image;
    ros::Subscriber _serial_sub;
    ros::Subscriber _bbox_sub;
    ros::Publisher _pub_map;
    cv::Mat _image;
    //CustomROISplitter _han;
    //------------------------
    GroupedROISplitter _han;
    //-----------------------------

    data _data;
    float _flag = 0;
    int _classes = 0;
    //int _fail_num = 0;
    bool _enable_image = false;
    bool _enable_yolo = false;

};


// 图像回调函数


int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "map_server_node");
    
    map_sever map;

    ros::spin();
    

    return 0;
}



