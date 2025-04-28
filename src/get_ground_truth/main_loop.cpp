#include "dual_camera.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <thread>

void printImageType(const cv::Mat& image, const std::string& name) {
    int type = image.type();
    std::string type_str;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth) {
        case CV_8U:  type_str = "8U"; break;
        case CV_8S:  type_str = "8S"; break;
        case CV_16U: type_str = "16U"; break;
        case CV_16S: type_str = "16S"; break;
        case CV_32S: type_str = "32S"; break;
        case CV_32F: type_str = "32F"; break;
        case CV_64F: type_str = "64F"; break;
        default:     type_str = "Unknown"; break;
    }

    type_str += "C" + std::to_string(chans);

    std::cout << "Image " << name << " type: " << type_str << std::endl;
}

void publishImage(DualCamera& camera, const std::string& topic, bool isRGB, bool isLeft, int frequency) {
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(topic, 1);

    ros::Rate loop_rate(frequency); // 设置发布频率

    while (ros::ok()) {
        cv::Mat image;
        if (isRGB) {
            image = isLeft ? camera.cameraL->getRGB() : camera.cameraR->getRGB();
            cv::flip(image, image, 1);
            image.convertTo(image, CV_8UC1);
            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        } else {
            image = isLeft ? camera.cameraL->getIxy() : camera.cameraR->getIxy();
        }

        if (!image.empty()) {
            std_msgs::Header header;
            header.stamp = ros::Time::now(); // 设置当前时间戳

            sensor_msgs::ImagePtr msg;
            if (isRGB) {
                msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
            } else {
                // 如果是浮点图像（如 CV_32FC1），使用 "32FC1" 编码格式
                if (image.type() == CV_32FC1) {
                    msg = cv_bridge::CvImage(header, "32FC1", image).toImageMsg();
                } else {
                    // 如果是单通道整型图像，使用 "mono8"
                    msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();
                }
            }
            pub.publish(msg);
            // ROS_INFO("Published %s image on topic: %s at time: %.9f", 
            //          isLeft ? "left" : "right", 
            //          topic.c_str(), 
            //          header.stamp.toSec());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char* argv[]) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "camera_image_publisher");

    DualCamera camera;
    camera.DataListener();

    // 创建四个线程，每个线程负责一个节点
    std::thread leftRGBThread(publishImage, std::ref(camera), "/camera/left/rgb", true, true, 30); // RGB 发布频率为 30 Hz
    std::thread rightRGBThread(publishImage, std::ref(camera), "/camera/right/rgb", true, false, 30);
    std::thread leftSDThread(publishImage, std::ref(camera), "/camera/left/sd", false, true, 400); // SD 发布频率为 400 Hz
    std::thread rightSDThread(publishImage, std::ref(camera), "/camera/right/sd", false, false, 400);

    // 等待线程结束
    leftRGBThread.join();
    rightRGBThread.join();
    leftSDThread.join();
    rightSDThread.join();

    return 0;
}
