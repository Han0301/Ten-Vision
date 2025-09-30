#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <libserial/SerialPort.h>
//#include "onnx.cpp"


//----------------------------------------------
// 获取帧尺寸（宽高）
int frame_width = static_cast<int>(640);
int frame_height = static_cast<int>(480);
std::string output_path = "/home/robocon/RC/project5_ws/src/yolopkg/video/basket4.mp4";
std::string sub_image_topic = "/camera/color/image_raw";
const char* fourcc = "mp4v"; // 或 "avc1"（H.264）、"h264"（需 FFmpeg 支持）
// 尝试打开视频写入器
bool is_color = true; // 是否为彩色视频（若输入是灰度图则设为 false）


//-----------------------------------------------------------------------------------------

cv::VideoWriter writer;
bool flag = false;



void save_video(cv::Mat& image)
{

    //保存视屏
    // cv::VideoCapture cap("/home/maple/study/project5_ws/src/yolopkg/video/test.mp4"); // 0 为默认摄像头，也可替换为视频文件路径（如 "input.mp4"）
    // if (!cap.isOpened()) {
    //     std::cerr << "错误：无法打开摄像头！" << std::endl;
    //     return;
    // }

    // 获取摄像头的实际帧率（可选，若需自定义可手动设置）
    // double fps = 30;
    // // if (fps <= 0) fps = 30; // 若摄像头帧率获取失败，默认设为 30 FPS

    // // 获取帧尺寸（宽高）
    // int frame_width = static_cast<int>(image.cols);
    // int frame_height = static_cast<int>(image.r);

    // // --------------------------
    // // 2. 配置视频写入器（输出）
    // // --------------------------
    // // 输出路径、FourCC、帧率、帧尺寸
    // std::string output_path = "/home/maple/study/project5_ws/src/yolopkg/video/test2.mp4";
    // const char* fourcc = "mp4v"; // 或 "avc1"（H.264）、"h264"（需 FFmpeg 支持）
    // //cv::VideoWriter writer;

    // // 尝试打开视频写入器
    // bool is_color = true; // 是否为彩色视频（若输入是灰度图则设为 false）
    // if (!writer.open(output_path, 
    //                 cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]), 
    //                 fps, 
    //                 cv::Size(frame_width, frame_height), 
    //                 is_color)) {
    //     std::cerr << "错误：无法创建视频写入器！请检查编码格式或路径权限。" << std::endl;
    //     return;
    // }

    // --------------------------
    // 3. 循环捕获并写入帧
    // --------------------------
    //cv::Mat frame;
    // int max_frames = 100; // 录制 100 帧后停止（可根据需求修改）
    // int frame_count = 0;

    // while (true) {
    //     if (!cap.read(frame)) {
    //         std::cerr << "错误：无法读取摄像头帧！" << std::endl;
    //         break;
    //     }

        // 可选：对帧进行处理（如缩放、滤波等）
        // cv::resize(frame, frame, cv::Size(640, 480)); // 调整帧尺寸（需与 writer 设置的尺寸一致）

        // 写入当前帧
    writer.write(image);
    std::cout<<"write"<<std::endl;
        // 显示实时画面（可选）
    //cv::imshow("Recording...", image);
    //cv::waitKey(30);
    // if (ros::ok()) { // 按 'q' 键提前终止录制
    //     std::cout << "用户手动终止录制。" << std::endl;
    //     writer.release();
    //     cv::destroyAllWindows();
    //     //break;
    //     flag = true;
    // }

        //frame_count++;
    //}

    // --------------------------
    // 4. 释放资源
    // --------------------------
    //cap.release();
    
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat image = cv_ptr->image;
        // // 处理图像（例如显示）
        // cv::namedWindow("img", cv::WINDOW_NORMAL);
        // //Bbox bbox;
        // //neural_network(image, bbox, my_onnx::net);
        // cv::imshow("img", image);
        // cv::waitKey(30);
        // std::cout << "recieve" << std::endl;
        save_video(image);




    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("转换失败: %s", e.what());
    }
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "sub_img_node");
    //my_onnx::net = cv::dnn::readNetFromONNX("src/rcpkg/onnx/best.onnx");
    // if (my_onnx::net.empty()) {
    //     std::cerr << "Error: Failed to load model. Possible reasons:" << std::endl;
    //     return -1;
    // }
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(sub_image_topic, 10, imageCallback);
    // 或者普通订阅者：ros::Subscriber sub = nh.subscribe("camera/image", 1, imageCallback);
    // 获取摄像头的实际帧率（可选，若需自定义可手动设置）
    double fps = 30;
    // if (fps <= 0) fps = 30; // 若摄像头帧率获取失败，默认设为 30 FPS



    // --------------------------
    // 2. 配置视频写入器（输出）
    // --------------------------
    // 输出路径、FourCC、帧率、帧尺寸
    
    //cv::VideoWriter writer;


    if (!writer.open(output_path, 
                    cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]), 
                    fps, 
                    cv::Size(frame_width, frame_height), 
                    is_color)) {
        std::cerr << "错误：无法创建视频写入器！请检查编码格式或路径权限。" << std::endl;
        return -1;
    }
    while(ros::ok())
    {
        //std::cout << "xxxx" << std::endl;
        
        ros::spinOnce();
    }
    writer.release();
    cv::destroyAllWindows();
    return 0;
}