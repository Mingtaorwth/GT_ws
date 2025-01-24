#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <mutex>
#include <iomanip>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/AccelStamped.h> 
namespace fs = std::filesystem;

// 清理目录
void clearDirectory(const std::string& folder_path) {
    if (fs::exists(folder_path)) {
        for (const auto& entry : fs::directory_iterator(folder_path)) {
            fs::remove(entry);
        }
        ROS_INFO("Cleared folder: %s", folder_path.c_str());
    }
}

// 清空文件内容
void clearFileIfExists(const std::string& filename) {
    std::ofstream file(filename, std::ios::out | std::ios::trunc);
    file.close();
}

#include <iomanip>  // 引入头文件来使用 std::setprecision

// 记录加速度数据到txt文件
void logAccelData(const ros::Time& timestamp, const geometry_msgs::TwistStamped::ConstPtr& accel_msg) {
    std::ofstream accel_file("/home/mingtao/Gt_ws/gt_acc.txt", std::ios_base::app);
    if (accel_file.is_open()) {
        accel_file << std::fixed << std::setprecision(9) << timestamp.toSec() << ", "  // 设置精度并使用固定小数点表示
                   << accel_msg->twist.linear.x << ", " << accel_msg->twist.linear.y << ", " << accel_msg->twist.linear.z << ", "
                   << accel_msg->twist.angular.x << ", " << accel_msg->twist.angular.y << ", " << accel_msg->twist.angular.z << std::endl;
        accel_file.close();
    }
}

// 记录位姿数据到txt文件
void logPoseData(const ros::Time& timestamp, const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
    std::ofstream pose_file("/home/mingtao/Gt_ws/gt_pose.txt", std::ios_base::app);
    if (pose_file.is_open()) {
        pose_file << std::fixed << std::setprecision(9) << timestamp.toSec() << ", "
                  << pose_msg->pose.position.x << ", " << pose_msg->pose.position.y << ", " << pose_msg->pose.position.z << ", "
                  << pose_msg->pose.orientation.x << ", " << pose_msg->pose.orientation.y << ", "
                  << pose_msg->pose.orientation.z << ", " << pose_msg->pose.orientation.w << std::endl;
        pose_file.close();
    }
}

// 记录速度数据到txt文件
void logTwistData(const ros::Time& timestamp, const geometry_msgs::TwistStamped::ConstPtr& twist_msg) {
    std::ofstream twist_file("/home/mingtao/Gt_ws/gt_twist.txt", std::ios_base::app);
    if (twist_file.is_open()) {
        twist_file << std::fixed << std::setprecision(9) << timestamp.toSec() << ", "
                   << twist_msg->twist.linear.x << ", " << twist_msg->twist.linear.y << ", " << twist_msg->twist.linear.z << ", "
                   << twist_msg->twist.angular.x << ", " << twist_msg->twist.angular.y << ", " << twist_msg->twist.angular.z << std::endl;
        twist_file.close();
    }
}

void logImuData(const ros::Time& timestamp, const sensor_msgs::Imu::ConstPtr& imu_msg) {
    std::ofstream imu_file("/home/mingtao/Gt_ws/Imu.txt", std::ios_base::app);
    if (imu_file.is_open()) {
        imu_file << std::fixed << std::setprecision(9) << timestamp.toSec() << ", "
                 << imu_msg->linear_acceleration.x << ", " << imu_msg->linear_acceleration.y << ", " << imu_msg->linear_acceleration.z << ", "
                 << imu_msg->angular_velocity.x << ", " << imu_msg->angular_velocity.y << ", " << imu_msg->angular_velocity.z << std::endl;
        imu_file.close();
    }
}


// 处理VRPN消息
void processVrpnMessage(const std::string& topic, const rosbag::MessageInstance& msg) {
    ros::Time timestamp = msg.getTime();

    if (topic == "/vrpn_client_node/Tracker0/accel") {
        geometry_msgs::TwistStamped::ConstPtr accel_msg = msg.instantiate<geometry_msgs::TwistStamped>();
        if (accel_msg) {
            logAccelData(timestamp, accel_msg);
        }
    } else if (topic == "/vrpn_client_node/Tracker0/pose") {
        geometry_msgs::PoseStamped::ConstPtr pose_msg = msg.instantiate<geometry_msgs::PoseStamped>();
        if (pose_msg) {
            logPoseData(timestamp, pose_msg);
        }
    } else if (topic == "/vrpn_client_node/Tracker0/twist") {
        geometry_msgs::TwistStamped::ConstPtr twist_msg = msg.instantiate<geometry_msgs::TwistStamped>();
        if (twist_msg) {
            logTwistData(timestamp, twist_msg);
        }
    } else if (topic == "/imu") {
    sensor_msgs::Imu::ConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
    if (imu_msg) {
        logImuData(timestamp, imu_msg);
        }
    } else {
        ROS_WARN("Unsupported VRPN topic: %s", topic.c_str());
    }
}

// 图像处理函数
void processSDImage(const cv::Mat& image, const std::string& topic,
                    const cv::Mat& map1_left, const cv::Mat& map2_left,
                    const cv::Mat& map1_right, const cv::Mat& map2_right) {
    cv::Mat map1, map2;
    std::string folder_path;

    static int frame_count_left_sd = 0;
    static int frame_count_right_sd = 0;
    int* frame_count = nullptr;

    if (topic == "/camera/left/sd") {
        folder_path = "/home/mingtao/Gt_ws/sdl";
        map1 = map1_left;
        map2 = map2_left;
        frame_count = &frame_count_left_sd;
    } else if (topic == "/camera/right/sd") {
        folder_path = "/home/mingtao/Gt_ws/sdr";
        map1 = map1_right;
        map2 = map2_right;
        frame_count = &frame_count_right_sd;
    }

    if (frame_count == nullptr) {
        ROS_WARN("Unsupported topic: %s", topic.c_str());
        return;
    }

    cv::Mat processed_image = image.clone();
    cv::flip(processed_image, processed_image, 1);
    cv::resize(processed_image, processed_image, cv::Size(320, 160), cv::INTER_LINEAR);

    double lowThreshold1 = -120;
    double highThreshold1 = -2;

    double lowThreshold2 = 2;
    double highThreshold2 = 120;

    cv::Mat mask1, mask2;
    cv::inRange(processed_image, lowThreshold1, highThreshold1, mask1);
    cv::inRange(processed_image, lowThreshold2, highThreshold2, mask2);

    cv::Mat combinedMask;
    cv::bitwise_or(mask1, mask2, combinedMask);

    cv::Mat filtered;
    processed_image.copyTo(filtered, combinedMask);

    // double minVal;
    // cv::minMaxLoc(filtered, &minVal, nullptr);

    // cv::Mat minMat = cv::Mat::ones(filtered.size(), filtered.type()) * static_cast<uchar>(minVal); 
    // cv::add(filtered, minMat, filtered);

    filtered = (filtered + 70) * 1.8;
    filtered.convertTo(filtered, CV_8UC1);
    cv::remap(filtered, filtered, map1, map2, cv::INTER_LINEAR);

    std::string filename = folder_path + "/frame_" + std::to_string((*frame_count)++) + ".bmp";
    cv::imwrite(filename, filtered);
    cv::imshow(topic + " - Processed", filtered);
    cv::waitKey(1);
}

// 处理图像数据
void processImage(const std::string& topic, const sensor_msgs::Image::ConstPtr& img_msg,
                  const cv::Mat& map1_left, const cv::Mat& map2_left,
                  const cv::Mat& map1_right, const cv::Mat& map2_right) {
    try {
        // 打印时间戳
        // ROS_INFO("Topic: %s, Timestamp: %.9f", topic.c_str(), img_msg->header.stamp.toSec());

        std::string encoding = img_msg->encoding;
        cv::Mat image;

        if (encoding == "bgr8") {
            image = cv_bridge::toCvShare(img_msg, "bgr8")->image;
        } else if (encoding == "mono8") {
            image = cv_bridge::toCvShare(img_msg, "mono8")->image;
        } else if (encoding == "32FC1") {
            image = cv_bridge::toCvShare(img_msg, "32FC1")->image;
        } else {
            ROS_WARN("Unsupported encoding: %s", encoding.c_str());
            return;
        }

        if (!image.empty()) {
            if (topic == "/camera/left/sd" || topic == "/camera/right/sd") {
                processSDImage(image, topic, map1_left, map2_left, map1_right, map2_right);
            } else {
                static int frame_count_left_rgb = 0;
                static int frame_count_right_rgb = 0;
                static std::mutex frame_mutex;
                cv::Mat map1, map2;
                std::string folder_path;

                if (topic == "/camera/left/rgb") {
                    folder_path = "/home/mingtao/Gt_ws/rgbl";
                    map1 = map1_left;
                    map2 = map2_left;

                    int frame_number;
                    {
                        std::lock_guard<std::mutex> lock(frame_mutex);
                        frame_number = frame_count_left_rgb++;
                    }

                    // 记录时间戳到文件，不使用FrameId
                    std::ofstream left_rgb_file("/home/mingtao/Gt_ws/left_rgb_timestamps.txt", std::ios_base::app);
                    if (left_rgb_file.is_open()) {
                        left_rgb_file << std::fixed << std::setprecision(9) << img_msg->header.stamp.toSec() << std::endl;
                        left_rgb_file.close();
                    }

                    std::string filename_rgb = folder_path + "/frame_" + std::to_string(frame_number) + ".bmp";
                    cv::remap(image, image, map1, map2, cv::INTER_LINEAR);
                    cv::imwrite(filename_rgb, image);
                    cv::imshow(topic + " - Processed", image);
                } else if (topic == "/camera/right/rgb") {
                    folder_path = "/home/mingtao/Gt_ws/rgbr";
                    map1 = map1_right;
                    map2 = map2_right;

                    int frame_number;
                    {
                        std::lock_guard<std::mutex> lock(frame_mutex);
                        frame_number = frame_count_right_rgb++;
                    }

                    std::string filename_rgb = folder_path + "/frame_" + std::to_string(frame_number) + ".bmp";
                    cv::remap(image, image, map1, map2, cv::INTER_LINEAR);
                    cv::imwrite(filename_rgb, image);
                    cv::imshow(topic + " - Processed", image);
                }
            }
        }

        // 记录时间戳到文件
        if (topic == "/camera/left/sd") {
            std::ofstream left_sd_file("/home/mingtao/Gt_ws/left_sd_timestamps.txt", std::ios_base::app);
            if (left_sd_file.is_open()) {
                left_sd_file << std::fixed << std::setprecision(9) << img_msg->header.stamp.toSec() << std::endl;
                left_sd_file.close();
            }
        }
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosbag_image_reader");

    cv::FileStorage fscv("/home/mingtao/THU/calibration/warping.xml", cv::FileStorage::READ);
    cv::Mat map1_left, map2_left, map1_right, map2_right;
    fscv["map1_left"] >> map1_left;
    fscv["map2_left"] >> map2_left;
    fscv["map1_right"] >> map1_right;
    fscv["map2_right"] >> map2_right;
    fscv.release();

    std::string bag_file;
    if (argc < 2) {
        std::cout << "Enter the path of the rosbag file: ";
        std::getline(std::cin, bag_file);  // 从控制台输入文件路径
    } else {
        bag_file = argv[1];  // 使用命令行参数
    }

    // 清理目录和文件
    clearDirectory("/home/mingtao/Gt_ws/sdl");
    clearDirectory("/home/mingtao/Gt_ws/sdr");
    clearDirectory("/home/mingtao/Gt_ws/rgbl");
    clearDirectory("/home/mingtao/Gt_ws/rgbr");
    clearFileIfExists("/home/mingtao/Gt_ws/left_sd_timestamps.txt");
    clearFileIfExists("/home/mingtao/Gt_ws/left_rgb_timestamps.txt");
    clearFileIfExists("/home/mingtao/Gt_ws/gt_acc.txt");
    clearFileIfExists("/home/mingtao/Gt_ws/gt_pose.txt");
    clearFileIfExists("/home/mingtao/Gt_ws/gt_twist.txt");
    clearFileIfExists("/home/mingtao/Gt_ws/Imu.txt");

    try {
        rosbag::Bag bag;
        bag.open(bag_file, rosbag::bagmode::Read);

        std::vector<std::string> topics = {
            "/camera/left/rgb",
            "/camera/right/rgb",
            "/camera/left/sd",
            "/camera/right/sd",
            "/vrpn_client_node/Tracker0/accel",
            "/vrpn_client_node/Tracker0/pose",
            "/vrpn_client_node/Tracker0/twist",
            "/imu"
        };

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        for (const rosbag::MessageInstance& msg : view) {
            sensor_msgs::Image::ConstPtr img_msg = msg.instantiate<sensor_msgs::Image>();
            if (img_msg) {
                processImage(msg.getTopic(), img_msg, map1_left, map2_left, map1_right, map2_right);
            } else {
                processVrpnMessage(msg.getTopic(), msg);  // 处理 VRPN 消息
            }
        }

        bag.close();
    } catch (const rosbag::BagException& e) {
        ROS_ERROR("Error reading bag file: %s", e.what());
    }

    return 0;
}
