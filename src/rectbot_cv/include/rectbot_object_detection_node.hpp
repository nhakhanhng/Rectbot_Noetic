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

#include "rectbot_cv/PoseObject.h"
#include "rectbot_cv/PoseObjectArray.h"

#include "yolov8.hpp"  // Include YOLOv8 header

class RectbotObjectDetectionNode {
public:
    RectbotObjectDetectionNode();
    ~RectbotObjectDetectionNode();

    bool initialize();
    void run();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_{"~"}; // Private node handle for parameters
    ros::Subscriber image_sub_;
    ros::Publisher processed_pub_;
    ros::Publisher detection_pub_;  // Publisher for detection results

    // YOLOv8 detector
    std::unique_ptr<YOLOv8> detector_;

    // Parameters
    std::string input_topic_;
    std::string output_topic_;
    std::string detection_topic_;
    std::string model_path_;
    float conf_threshold_;
    float nms_threshold_;
    // float conf_threshold_
    
    // Image callback
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    
    // Object detection
    bool detectObjects(const cv::Mat& image, std::vector<cv::Rect>& detections, 
                                        std::vector<float>& confidences, std::vector<int>& class_ids);
    
    // Load the detection model
    bool loadModel();

    void loadParameters();
    
    // Model variables
    cv::dnn::Net network_;
    std::vector<std::string> class_names_;
};


#endif // RECTBOT_OBJECT_DETECTION_NODE_HPP