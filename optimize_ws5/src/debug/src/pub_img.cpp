#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


//-----------------------------------------------------------------------------------------
std::string video_path = "/home/maple/study2/optimize_ws4/src/debug/video/wqj.mp4"; 
cv::Size image_size(1920, 1080);
std::string pub_image_topic = "/camera/color/image_raw";

//-----------------------------------------------------------------------------------------

void publish_img()
{
    ros::NodeHandle nh("/");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(pub_image_topic, 1);
    // 或者使用普通发布者：ros::Publisher pub = nh.advertise<sensor_msgs::Image>("camera/image", 1);

    //cv::VideoCapture cap(2,cv::CAP_V4L2); // 打开摄像头  
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cv::VideoCapture cap(video_path);
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    if (!cap.isOpened()) {
        ROS_ERROR("无法打开摄像头！");
        return ;
    }


    //发布图像
    ros::Rate rate(30); // 发布频率40Hz
    while (nh.ok()) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) continue;
        cv::resize(frame, frame, image_size);
        // 转换为ROS消息
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
        std::cout << "publish success" << std::endl;
        ros::spinOnce();
        rate.sleep();
    }
    cap.release();
}


void save_video()
{
    //保存视屏
    cv::VideoCapture cap("/home/maple/study/project5_ws/src/yolopkg/video/test.mp4"); // 0 为默认摄像头，也可替换为视频文件路径（如 "input.mp4"）
    if (!cap.isOpened()) {
        std::cerr << "错误：无法打开摄像头！" << std::endl;
        return;
    }

    // 获取摄像头的实际帧率（可选，若需自定义可手动设置）
    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 0) fps = 30; // 若摄像头帧率获取失败，默认设为 30 FPS

    // 获取帧尺寸（宽高）
    int frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    // --------------------------
    // 2. 配置视频写入器（输出）
    // --------------------------
    // 输出路径、FourCC、帧率、帧尺寸
    std::string output_path = "/home/robocon/RC/project5_ws/src/yolopkg/video/test2.mp4";
    const char* fourcc = "mp4v"; // 或 "avc1"（H.264）、"h264"（需 FFmpeg 支持）
    cv::VideoWriter writer;

    // 尝试打开视频写入器
    bool is_color = true; // 是否为彩色视频（若输入是灰度图则设为 false）
    if (!writer.open(output_path, 
                    cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]), 
                    fps, 
                    cv::Size(frame_width, frame_height), 
                    is_color)) {
        std::cerr << "错误：无法创建视频写入器！请检查编码格式或路径权限。" << std::endl;
        return;
    }

    // --------------------------
    // 3. 循环捕获并写入帧
    // --------------------------
    cv::Mat frame;
    // int max_frames = 100; // 录制 100 帧后停止（可根据需求修改）
    // int frame_count = 0;

    while (true) {
        if (!cap.read(frame)) {
            std::cerr << "错误：无法读取摄像头帧！" << std::endl;
            break;
        }

        // 可选：对帧进行处理（如缩放、滤波等）
        // cv::resize(frame, frame, cv::Size(640, 480)); // 调整帧尺寸（需与 writer 设置的尺寸一致）

        // 写入当前帧
        writer.write(frame);

        // 显示实时画面（可选）
        cv::imshow("Recording...", frame);
        if (cv::waitKey(30) == 'q') { // 按 'q' 键提前终止录制
            std::cout << "用户手动终止录制。" << std::endl;
            break;
        }

        //frame_count++;
    }

    // --------------------------
    // 4. 释放资源
    // --------------------------
    cap.release();
    writer.release();
    cv::destroyAllWindows();
}


int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "pub_img_node");
    publish_img();
    //save_video();
    return 0;
}