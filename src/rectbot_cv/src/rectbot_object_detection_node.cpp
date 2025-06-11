// rectbot_object_detection_node.cpp
#include "rectbot_object_detection_node.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>

RectbotObjectDetectionNode::RectbotObjectDetectionNode()
    : private_nh_("~")
{
    loadParameters();
    ROS_INFO("Parameters loaded:");
    ROS_INFO("  Input topic: %s", input_topic_.c_str());
    ROS_INFO("  Output topic: %s", output_topic_.c_str());
    ROS_INFO("  Detection topic: %s", detection_topic_.c_str());
    ROS_INFO("  Model path: %s", model_path_.c_str());
    ROS_INFO("  Confidence threshold: %.2f", conf_threshold_);
    ROS_INFO("  NMS threshold: %.2f", nms_threshold_);

    cudaSetDevice(0);
    detector_ = std::make_unique<YOLOv8>(model_path_);
    ROS_INFO("YOLOv8 model loaded");

    image_sub_     = nh_.subscribe(input_topic_, 1, &RectbotObjectDetectionNode::imageCallback, this);
    processed_pub_ = nh_.advertise<sensor_msgs::Image>(output_topic_, 1);
    detection_pub_ = nh_.advertise<rectbot_cv::PoseObjectArray>(detection_topic_, 1);
    ROS_INFO("RectbotObjectDetectionNode initialized with YOLO model: %s", model_path_.c_str());

    detector_->MakePipe(true);
    process_d2c_thread_ = std::thread(&RectbotObjectDetectionNode::detectObjects_thread, this);
}

RectbotObjectDetectionNode::~RectbotObjectDetectionNode()
{
    running_ = false;
    if (process_d2c_thread_.joinable()) process_d2c_thread_.join();
    ROS_INFO("RectbotObjectDetectionNode destroyed");
}

void RectbotObjectDetectionNode::loadParameters()
{
    private_nh_.param<std::string>("ros/color_image_topic",       input_topic_,       "camera/image_raw");
    private_nh_.param<std::string>("ros/output_topic",            output_topic_,      "processed_image");
    private_nh_.param<std::string>("ros/detection_topic",         detection_topic_,   "detections");
    private_nh_.param<std::string>("model/weights",               model_path_,        "./models/yolov8n.engine");
    private_nh_.param<float>("detection/confidence_threshold", conf_threshold_,    0.25f);
    private_nh_.param<float>("detection/nms_threshold",        nms_threshold_,     0.45f);
}

void RectbotObjectDetectionNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(image_mutex_);
    image_msg_ = msg;
}

void RectbotObjectDetectionNode::detectObjects_thread()
{
    // sensor_msgs::ImageConstPtr last_processed_msg;
    while (ros::ok() && running_) {

        if (!image_msg_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        sensor_msgs::ImageConstPtr msg_copy;
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            // if (!image_msg_ || image_msg_ == last_processed_msg) {
            //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
            //     continue;
            // }
            msg_copy = image_msg_;
        }

        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_copy, sensor_msgs::image_encodings::BGR8);
            auto start = std::chrono::high_resolution_clock::now();

            detector_->CopyFromMat(cv_ptr->image);
            detector_->Infer();
            std::vector<det::PoseObject> objs;
            detector_->PostProcessPose(objs, conf_threshold_, nms_threshold_, 100, 80);

            auto end = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            float fps = elapsed > 0 ? 1000.0f / elapsed : 0.0f;

            cv::Mat output_image = cv_ptr->image.clone();
            std::string fps_text = "FPS: " + std::to_string(fps);
            cv::putText(output_image, fps_text, cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,255,0), 2);
            detector_->DrawPoseObjects(output_image, objs, 0.7);

            cv_bridge::CvImage out_msg;
            out_msg.header   = msg_copy->header;
            out_msg.encoding = sensor_msgs::image_encodings::BGR8;
            out_msg.image    = output_image;
            processed_pub_.publish(out_msg.toImageMsg());

            rectbot_cv::PoseObjectArray pose_obj_array_msg;
            for (auto& obj : objs) {
                rectbot_cv::PoseObject pmsg;
                pmsg.detection.header     = msg_copy->header;
                pmsg.detection.bbox.center.x = obj.rect.x + obj.rect.width * 0.5f;
                pmsg.detection.bbox.center.y = obj.rect.y + obj.rect.height * 0.5f;
                pmsg.detection.bbox.size_x = obj.rect.width;
                pmsg.detection.bbox.size_y = obj.rect.height;
                for (auto& kp : obj.keypoints) {
                    rectbot_cv::KeyPoint kpt;
                    kpt.x.data     = kp.x;
                    kpt.y.data     = kp.y;
                    kpt.score.data = kp.score;
                    pmsg.keypoints.push_back(kpt);
                }
                vision_msgs::ObjectHypothesisWithPose hyp;
                hyp.id    = obj.label;
                hyp.score = obj.prob;
                pmsg.detection.results.push_back(hyp);
                pose_obj_array_msg.objects.push_back(pmsg);
            }
            detection_pub_.publish(pose_obj_array_msg);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        // last_processed_msg = msg_copy;
        // {
        //     std::lock_guard<std::mutex> lock(image_mutex_);
        //     image_msg_.reset();
        // }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rectbot_object_detection_node");
    RectbotObjectDetectionNode node;
    ros::spin();
    return 0;
}
