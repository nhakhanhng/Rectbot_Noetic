#ifndef RECTBOT_OBJECT_DETECTION_NODE_HPP
#define RECTBOT_OBJECT_DETECTION_NODE_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <vision_msgs/Detection2DArray.h>
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>

#include <thread>
#include <mutex>
#include <atomic>

#include "rectbot_cv/PoseObject.h"
#include "rectbot_cv/PoseObjectArray.h"

#include "yolov8.hpp"  // Include YOLOv8 header

class RectbotObjectDetectionNode {
public:
    RectbotObjectDetectionNode();
    ~RectbotObjectDetectionNode();

private:
    void loadParameters();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void detectObjects_thread();

    ros::NodeHandle nh_, private_nh_;
    std::string input_topic_, output_topic_, detection_topic_, model_path_;
    float conf_threshold_, nms_threshold_;
    std::unique_ptr<YOLOv8> detector_;
    sensor_msgs::ImageConstPtr image_msg_;

    // threading
    std::thread process_d2c_thread_;
    std::mutex image_mutex_;
    std::atomic<bool> running_{true};

    ros::Subscriber image_sub_;
    ros::Publisher processed_pub_;
    ros::Publisher detection_pub_;
};


#endif // RECTBOT_OBJECT_DETECTION_NODE_HPP