

#include "rectbot_object_detection_node.hpp"



RectbotObjectDetectionNode::RectbotObjectDetectionNode() {
    // Load parameters
    loadParameters();
    // Log loaded parameters
    ROS_INFO("Parameters loaded:");
    ROS_INFO("  Input topic: %s", input_topic_.c_str());
    ROS_INFO("  Output topic: %s", output_topic_.c_str());
    ROS_INFO("  Detection topic: %s", detection_topic_.c_str());
    ROS_INFO("  Model path: %s", model_path_.c_str());
    ROS_INFO("  Confidence threshold: %.2f", conf_threshold_);
    ROS_INFO("  NMS threshold: %.2f", nms_threshold_);
    // Initialize YOLOv8 detector
    cudaSetDevice(0);
    detector_ = std::make_unique<YOLOv8>(model_path_);
    ROS_INFO("YOLOv8 model loaded");
    // Initialize subscribers and publishers
    image_sub_ = nh_.subscribe(input_topic_, 1, &RectbotObjectDetectionNode::imageCallback, this);
    processed_pub_ = nh_.advertise<sensor_msgs::Image>(output_topic_, 1);
    detection_pub_ = nh_.advertise<rectbot_cv::PoseObjectArray>(detection_topic_, 1);
    
    ROS_INFO("RectbotObjectDetectionNode initialized with YOLO model: %s", model_path_.c_str());
    detector_->MakePipe(true);

    // Create a thread to run processD2C
    process_d2c_thread_ = std::thread(&RectbotObjectDetectionNode::detectObjects_thread, this);

}

void RectbotObjectDetectionNode::detectObjects_thread() {
    sensor_msgs::ImageConstPtr last_processed_msg = nullptr; // Keep track of the last processed image message

    while (ros::ok()) {
        if (image_msg_ != nullptr && image_msg_ != last_processed_msg) {
            try {
                // Convert ROS image message to OpenCV image
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg_, sensor_msgs::image_encodings::BGR8);

                // Start time measurement for FPS calculation
                auto start = std::chrono::high_resolution_clock::now();

                // Run object detection
                detector_->CopyFromMat(cv_ptr->image);
                detector_->Infer();
                std::vector<det::PoseObject> objs;
                detector_->PostProcessPose(objs, conf_threshold_, nms_threshold_, 100, 80);

                // End time measurement and calculate FPS
                auto end = std::chrono::high_resolution_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                float fps = 1000.0 / elapsed;

                // Draw detections on the image
                cv::Mat output_image = cv_ptr->image.clone();
                std::string fps_text = "FPS: " + std::to_string(fps);
                cv::putText(output_image, fps_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                            1.0, cv::Scalar(0, 255, 0), 2);
                detector_->DrawPoseObjects(output_image, objs, 0.7);

                // Publish processed image
                cv_bridge::CvImage out_msg;
                out_msg.header = image_msg_->header;
                out_msg.encoding = sensor_msgs::image_encodings::BGR8;
                out_msg.image = output_image;
                processed_pub_.publish(out_msg.toImageMsg());

                // Publish detection results
                rectbot_cv::PoseObjectArray pose_obj_array_msg;
                for (auto& obj : objs) {
                    rectbot_cv::PoseObject pose_obj_msg;
                    pose_obj_msg.detection.header = image_msg_->header;
                    pose_obj_msg.detection.bbox.center.x = obj.rect.x + obj.rect.width / 2;
                    pose_obj_msg.detection.bbox.center.y = obj.rect.y + obj.rect.height / 2;
                    pose_obj_msg.detection.bbox.size_x = obj.rect.width;
                    pose_obj_msg.detection.bbox.size_y = obj.rect.height;

                    for (auto& kp : obj.keypoints) {
                        rectbot_cv::KeyPoint kpt;
                        kpt.x.data = kp.x;
                        kpt.y.data = kp.y;
                        kpt.score.data = kp.score;
                        pose_obj_msg.keypoints.push_back(kpt);
                    }

                    vision_msgs::ObjectHypothesisWithPose hypothesis;
                    hypothesis.id = obj.label;
                    hypothesis.score = obj.prob;
                    pose_obj_msg.detection.results.push_back(hypothesis);
                    pose_obj_array_msg.objects.push_back(pose_obj_msg);
                }

                detection_pub_.publish(pose_obj_array_msg);

                // Update the last processed message
                last_processed_msg = image_msg_;

            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }

            // Reset the image message after processing
            image_msg_ = nullptr;
        }
    }
}

RectbotObjectDetectionNode::~RectbotObjectDetectionNode() {
    // Destructor
    ROS_INFO("RectbotObjectDetectionNode destroyed");
}

void RectbotObjectDetectionNode::loadParameters() {
    // Load parameters with defaults
    private_nh_.param<std::string>("/rectbot_object_detection_node/ros/color_image_topic", input_topic_, "camera/image_raw");
    private_nh_.param<std::string>("/rectbot_object_detection_node/ros/output_topic", output_topic_, "processed_image");
    private_nh_.param<std::string>("/rectbot_object_detection_node/ros/detection_topic", detection_topic_, "detections");
    private_nh_.param<std::string>("/rectbot_object_detection_node/model/weights", model_path_, "./models/yolov8n.engine");
    private_nh_.param<float>("/rectbot_object_detection_node/detection/confidence_threshold", conf_threshold_, 0.25f);
    private_nh_.param<float>("/rectbot_object_detection_node/detection/nms_threshold", nms_threshold_, 0.45f);
}

void RectbotObjectDetectionNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {

    image_msg_ = msg;
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rectbot_object_detection_node");
    
    RectbotObjectDetectionNode node;
    
    ros::spin();
    
    return 1;
}