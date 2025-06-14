#include "rectbot_object_mapping_node.hpp"

#include "geometry_msgs/PointStamped.h"

// #include 


RectbotObjectMapper::RectbotObjectMapper():
    tf_listener_(tf_buffer_),tracker_(), running_(true), markers_id_(0)
{
    loadParameters();
    // Print all loaded parameters
    std::cout << "=== Loaded Parameters ===" << std::endl;
    std::cout << "depth_frame_id: " << depth_frame_id_ << std::endl;
    std::cout << "color_frame_id: " << color_frame_id_ << std::endl;
    std::cout << "depth2color_aligned_frame_id: " << depth2color_aligned_frame_id_ << std::endl;
    std::cout << "base_frame_id: " << base_frame_id_ << std::endl;
    std::cout << "object_frame_id: " << object_frame_id_ << std::endl;
    std::cout << "depth2color_aligned_topic: " << depth2color_aligned_topic_ << std::endl;
    std::cout << "depth_image_topic: " << depth_image_topic_ << std::endl;
    std::cout << "color_image_topic: " << color_image_topic_ << std::endl;
    std::cout << "detection_topic: " << detection_topic_ << std::endl;
    std::cout << "depth_info_topic: " << depth_info_topic_ << std::endl;
    std::cout << "color_info_topic: " << color_info_topic_ << std::endl;
    std::cout << "depth2color_aligned_info_topic: " << depth2color_aligned_info_topic_ << std::endl;
    std::cout << "object_marker_topic: " << object_marker_topic_ << std::endl;
    std::cout << "========================" << std::endl;
    loadCalibrationMatrix();
    // logCalibrationMatrix();
    depth_image_sub_ = nh_.subscribe(depth_image_topic_, 1, &RectbotObjectMapper::depthImageCallback, this);
    depth2color_aligned_sub_ = nh_.subscribe(depth2color_aligned_topic_, 1, &RectbotObjectMapper::depth2colorAlignedCallback, this);
    detection_sub_ = nh_.subscribe(detection_topic_, 1, &RectbotObjectMapper::detectionCallback, this);
    depth_info_sub_ = nh_.subscribe(depth_info_topic_, 1, &RectbotObjectMapper::depthImageInfoCallback, this);
    color_info_sub_ = nh_.subscribe(color_info_topic_, 1, &RectbotObjectMapper::colorImageInfoCallback, this);
    depth2color_aligned_info_sub_ = nh_.subscribe(depth2color_aligned_info_topic_, 1, &RectbotObjectMapper::depth2colorAlignedInfoCallback, this);
    object_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(object_marker_topic_, 1);

    // Create a timer to periodically run processD2CAligned
    // process_timer_ = nh_.createTimer(ros::Duration(0.4), [this](const ros::TimerEvent&) {
    //     processD2CAligned();
    // });

    markers_id_ = 0;

    process_thread_ = std::thread(&RectbotObjectMapper::threadedProcess, this);
}

RectbotObjectMapper::~RectbotObjectMapper()
{
    // Clear all markers
    visualization_msgs::MarkerArray clear_markers_msg;
    visualization_msgs::Marker clear_marker;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    clear_markers_msg.markers.push_back(clear_marker);
    object_marker_pub_.publish(clear_markers_msg);
    // Destructor
    depth_image_sub_.shutdown();
    depth2color_aligned_sub_.shutdown();
    detection_sub_.shutdown();
    depth_info_sub_.shutdown();
    color_info_sub_.shutdown();
    depth2color_aligned_info_sub_.shutdown();
    object_marker_pub_.shutdown();

    if (process_thread_.joinable()) {
        running_ = false;
        process_thread_.join();
    }

    ROS_INFO("RectbotObjectMapper destroyed");


}

void RectbotObjectMapper::loadParameters()
{
    private_nh_.param<std::string>("/rectbot_object_mapping_node/ros/depth_frame_id", depth_frame_id_, "camera_depth_optical_frame");
    private_nh_.param<std::string>("/rectbot_object_mapping_node/ros/color_frame_id", color_frame_id_, "camera_color_optical_frame");
    private_nh_.param<std::string>("/rectbot_object_mapping_node/ros/depth2color_aligned_frame_id", depth2color_aligned_frame_id_, "camera_aligned_depth_to_color_frame");
    private_nh_.param<std::string>("/rectbot_object_mapping_node/ros/base_frame_id", base_frame_id_, "base_link");
    private_nh_.param<std::string>("/rectbot_object_mapping_node/ros/object_frame_id", object_frame_id_, "object_frame");
    private_nh_.param<std::string>("/rectbot_object_mapping_node/ros/depth2color_aligned_topic", depth2color_aligned_topic_, "/camera/aligned_depth_to_color/image_raw");
    private_nh_.param<std::string>("/rectbot_object_mapping_node/ros/depth_image_topic", depth_image_topic_, "/camera/depth/image_raw");
    private_nh_.param<std::string>("/rectbot_object_mapping_node/ros/color_image_topic", color_image_topic_, "/camera/color/image_raw");
    private_nh_.param<std::string>("/rectbot_object_mapping_node/ros/alinged_depth_topic", depth2color_aligned_topic_, "/camera/aligned_depth_to_color/image_raw");
    private_nh_.param<std::string>("/rectbot_object_mapping_node/ros/aligned_depth_info_topic", depth2color_aligned_info_topic_, "/camera/aligned_depth_to_color/camera_info");
    private_nh_.param<std::string>("/rectbot_object_mapping_node/ros/detection_topic", detection_topic_, "/object_detection");
    private_nh_.param<std::string>("/rectbot_object_mapping_node/ros/depth_info_topic", depth_info_topic_, "/camera/depth/camera_info");
    private_nh_.param<std::string>("/rectbot_object_mapping_node/ros/color_info_topic", color_info_topic_, "/camera/color/camera_info");
    private_nh_.param<std::string>("/rectbot_object_mapping_node/ros/object_marker_topic", object_marker_topic_, "/object_markers");

}

void RectbotObjectMapper::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    depth_image_msg_ = msg;
}

void RectbotObjectMapper::depth2colorAlignedCallback(const sensor_msgs::ImageConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    depth2color_aligned_image_msg_ = msg;
}


void RectbotObjectMapper::depth2colorAlignedInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    depth2color_aligned_camera_model_.fromCameraInfo(msg);
}


void RectbotObjectMapper::threadedProcess() {
    while (ros::ok() && running_) {
        
            // std::lock_guard<std::mutex> lock(data_mutex_);
        processD2CAligned();
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}


void RectbotObjectMapper::detectionCallback(const rectbot_cv::PoseObjectArrayConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    detections_msg_ = msg->objects;
    // processDetections();
    // ROS_INFO("Received %zu detections", detections_msg_.size());
    // processD2CAligned();
}

void RectbotObjectMapper::depthImageInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    depth_camera_model_.fromCameraInfo(msg);
}

void RectbotObjectMapper::colorImageInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    color_camera_model_.fromCameraInfo(msg);
}

void RectbotObjectMapper::getExtrinsicMatrix(rs2_extrinsics &extrinsic_matrix,std::string param_name)
{
    // Attempt to read extrinsic parameters from parameter server
    std::vector<float> rotation_matrix;
    std::vector<float> translation_vector;
    // float *rotation_matrix = new float[9];
    // float *translation_vector = new float[3];

    if (!private_nh_.getParam("/rectbot_object_mapping_node/calibration/extrinsics/" + param_name + "/rotation_matrix",rotation_matrix)) {
        ROS_WARN("Failed to get rotation matrix from parameter server or invalid size (expected 9 elements)");
        // Set identity rotation as fallback
        for (int i = 0; i < 9; i++) {
            extrinsic_matrix.rotation[i] = (i == 0 || i == 4 || i == 8) ? 1.0f : 0.0f;
        }
    } else {
        // Copy rotation matrix values
        for (int i = 0; i < 9; i++) {
            extrinsic_matrix.rotation[i] = static_cast<float>(rotation_matrix[i]);
        }
    }

    if (!private_nh_.getParam("/rectbot_object_mapping_node/calibration/extrinsics/" +  param_name + "/translation_vector", translation_vector)) {
        ROS_WARN("Failed to get translation vector from parameter server or invalid size (expected 3 elements)");
        // Set zero translation as fallback
        for (int i = 0; i < 3; i++) {
            extrinsic_matrix.translation[i] = 0.0f;
        }
    } else {
        // Copy translation vector values
        for (int i = 0; i < 3; i++) {
            extrinsic_matrix.translation[i] = static_cast<float>(translation_vector[i]);
        }
    }


    ROS_INFO_STREAM("Loaded extrinsic matrix from " << param_name);
}


void RectbotObjectMapper::getIntrinsicMatrix(rs2_intrinsics &intrinsic_matrix,std::string param_name) {
    // Attempt to read intrinsic parameters from the structured parameter server
    int width, height, model_id;
    double fx, fy, cx, cy;
    double k1, k2, p1, p2, k3;
    std::string param_prefix = "/rectbot_object_mapping_node/calibration/intrinsics/" + param_name + "/";
    
    // Read basic camera parameters
    if (!private_nh_.getParam(param_prefix + "width", width) ||
        !private_nh_.getParam(param_prefix + "height", height) ||
        !private_nh_.getParam(param_prefix + "fx", fx) ||
        !private_nh_.getParam(param_prefix + "fy", fy) ||
        !private_nh_.getParam(param_prefix + "cx", cx) ||
        !private_nh_.getParam(param_prefix + "cy", cy) ||
        !private_nh_.getParam(param_prefix + "model_id", model_id)) {
        
        ROS_WARN("Failed to get all required intrinsic parameters for %s", param_name.c_str());
        // Set default intrinsic matrix as fallback
        intrinsic_matrix.width = 640;
        intrinsic_matrix.height = 480;
        intrinsic_matrix.ppx = 320;
        intrinsic_matrix.ppy = 240;
        intrinsic_matrix.fx = 500;
        intrinsic_matrix.fy = 500;
        intrinsic_matrix.model = RS2_DISTORTION_NONE;
        for (int i = 0; i < 5; i++) {
            intrinsic_matrix.coeffs[i] = 0.0f;
        }
    } else {
        // Read the distortion coefficients (with defaults if not found)
        private_nh_.param<double>(param_prefix + "k1", k1, 0.0);
        private_nh_.param<double>(param_prefix + "k2", k2, 0.0);
        private_nh_.param<double>(param_prefix + "p1", p1, 0.0);
        private_nh_.param<double>(param_prefix + "p2", p2, 0.0);
        private_nh_.param<double>(param_prefix + "k3", k3, 0.0);
        
        // Set the intrinsic parameters
        intrinsic_matrix.width = width;
        intrinsic_matrix.height = height;
        intrinsic_matrix.fx = static_cast<float>(fx);
        intrinsic_matrix.fy = static_cast<float>(fy);
        intrinsic_matrix.ppx = static_cast<float>(cx);
        intrinsic_matrix.ppy = static_cast<float>(cy);
        intrinsic_matrix.model = static_cast<rs2_distortion>(model_id);
        
        // Set distortion coefficients
        intrinsic_matrix.coeffs[0] = static_cast<float>(k1);
        intrinsic_matrix.coeffs[1] = static_cast<float>(k2);
        intrinsic_matrix.coeffs[2] = static_cast<float>(p1);
        intrinsic_matrix.coeffs[3] = static_cast<float>(p2);
        intrinsic_matrix.coeffs[4] = static_cast<float>(k3);
    }
    ROS_INFO_STREAM("Loaded intrinsic matrix from " << param_name);
}
void RectbotObjectMapper::loadCalibrationMatrix()
{
    getIntrinsicMatrix(depth_intrinsics_, "depth");
    getIntrinsicMatrix(color_intrinsics_, "color");
    getExtrinsicMatrix(color2depth_extrinsic_, "color2depth");
    getExtrinsicMatrix(depth2color_extrinsic_, "depth2color");
}

void RectbotObjectMapper::logCalibrationMatrix()
{
    // Log all calibration matrices
    ROS_INFO("===== Camera Calibration Matrices =====");

    // Log depth intrinsics
    ROS_INFO("Depth Intrinsics:");
    ROS_INFO("Resolution: %dx%d", depth_intrinsics_.width, depth_intrinsics_.height);
    ROS_INFO("Principal Point (px, py): %.8f, %.8f", depth_intrinsics_.ppx, depth_intrinsics_.ppy);
    ROS_INFO("Focal Length (fx, fy): %.8f, %.8f", depth_intrinsics_.fx, depth_intrinsics_.fy);
    ROS_INFO("Distortion Model: %d", depth_intrinsics_.model);
    ROS_INFO("Distortion Coeffs: [%.8f, %.8f, %.8f, %.8f, %.8f]",
        depth_intrinsics_.coeffs[0], depth_intrinsics_.coeffs[1], depth_intrinsics_.coeffs[2],
        depth_intrinsics_.coeffs[3], depth_intrinsics_.coeffs[4]);

    // Log color intrinsics
    ROS_INFO("Color Intrinsics:");
    ROS_INFO("Resolution: %dx%d", color_intrinsics_.width, color_intrinsics_.height);
    ROS_INFO("Principal Point (px, py): %.8f, %.8f", color_intrinsics_.ppx, color_intrinsics_.ppy);
    ROS_INFO("Focal Length (fx, fy): %.8f, %.8f", color_intrinsics_.fx, color_intrinsics_.fy);
    ROS_INFO("Distortion Model: %d", color_intrinsics_.model);
    ROS_INFO("Distortion Coeffs: [%.8f, %.8f, %.8f, %.8f, %.8f]",
        color_intrinsics_.coeffs[0], color_intrinsics_.coeffs[1], color_intrinsics_.coeffs[2],
        color_intrinsics_.coeffs[3], color_intrinsics_.coeffs[4]);

    // Log color2depth extrinsics
    ROS_INFO("Color to Depth Extrinsics:");
    ROS_INFO("Rotation Matrix:");
    ROS_INFO("[%.8f, %.8f, %.8f]", color2depth_extrinsic_.rotation[0], 
        color2depth_extrinsic_.rotation[1], color2depth_extrinsic_.rotation[2]);
    ROS_INFO("[%.8f, %.8f, %.8f]", color2depth_extrinsic_.rotation[3], 
        color2depth_extrinsic_.rotation[4], color2depth_extrinsic_.rotation[5]);
    ROS_INFO("[%.8f, %.8f, %.8f]", color2depth_extrinsic_.rotation[6], 
        color2depth_extrinsic_.rotation[7], color2depth_extrinsic_.rotation[8]);
    ROS_INFO("Translation Vector: [%.8f, %.8f, %.8f]", 
        color2depth_extrinsic_.translation[0], color2depth_extrinsic_.translation[1], 
        color2depth_extrinsic_.translation[2]);

    // Log depth2color extrinsics
    ROS_INFO("Depth to Color Extrinsics:");
    ROS_INFO("Rotation Matrix:");
    ROS_INFO("[%.8f, %.8f, %.8f]", depth2color_extrinsic_.rotation[0], 
        depth2color_extrinsic_.rotation[1], depth2color_extrinsic_.rotation[2]);
    ROS_INFO("[%.8f, %.8f, %.8f]", depth2color_extrinsic_.rotation[3], 
        depth2color_extrinsic_.rotation[4], depth2color_extrinsic_.rotation[5]);
    ROS_INFO("[%.8f, %.8f, %.8f]", depth2color_extrinsic_.rotation[6], 
        depth2color_extrinsic_.rotation[7], depth2color_extrinsic_.rotation[8]);
    ROS_INFO("Translation Vector: [%.8f, %.8f, %.8f]", 
        depth2color_extrinsic_.translation[0], depth2color_extrinsic_.translation[1],
        depth2color_extrinsic_.translation[2]);

    ROS_INFO("==================================");
}

void RectbotObjectMapper::processDetections()
{
    if (!depth_image_msg_ || detections_msg_.empty() || !depth_camera_model_.initialized() || !color_camera_model_.initialized())
    {
        // ROS_WARN_THROTTLE(1.0, "Skipping detection processing. Missing data.");
        return;
    }
    visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
    visualization_msgs::Marker m;
    m.header.frame_id = depth_frame_id_;
    m.header.stamp = ros::Time::now();
    m.ns = "objects";
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.color.r = 0;
    m.color.g = 0;
    m.color.b = 255;
    m.color.a = 255;
     // lives forever
    m.lifetime = ros::Duration(0);
    m.frame_locked = true;

    //get depth image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(depth_image_msg_, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat depth_image = cv_ptr->image;
    // Now we can process the detections with the extrinsic matrix
    for (auto det : detections_msg_) {
        // float dpt_bbox_center_pixel[2] = {0};
        // float rgb_bbox_center_pixel[2] = {det.bbox.center.x, det.bbox.center.y};
        // rs2_project_color_pixel_to_depth_pixel(dpt_bbox_center_pixel, reinterpret_cast<const uint16_t*>(depth_frame.get_data()), depth_frame.get_units(), 0.1f, 10,
        //             &depth_intrinsics_, &color_intrinsics_,
        //             &color2depth_extrinsic_ , &depth2color_extrinsic_ , rgb_bbox_center_pixel);
        // // Get the depth value at the center of the bounding box
        // float depth = depth_image.at<uint16_t>(dpt_bbox_center_pixel[1], dpt_bbox_center_pixel[0]) * 1000;
        // // Get the 3D coordinates of the center of the bounding box
        // float point[3] = {0};
        // rs2_deproject_pixel_to_point(point, &depth_intrinsics_, dpt_bbox_center_pixel, depth);
        // // Create a marker for the object
        // m.id = id++;
        // m.type = visualization_msgs::Marker::CUBE;
        // m.action = visualization_msgs::Marker::ADD;
        // m.pose.position.x = point[0];
        // m.pose.position.y = point[1];
        // m.pose.position.z = point[2];
        // m.pose.orientation.x = 0;
        // m.pose.orientation.y = 0;
        // m.pose.orientation.z = 0;
        // m.pose.orientation.w = 1;
        // // m.scale.x = det.bbox.size.x;
        // // m.scale.y = det.bbox.size.y;
        // // m.scale.z = det.bbox.size.z;
        // markers.push_back(m);
    }
}

bool RectbotObjectMapper::isOldObject(geometry_msgs::PointStamped position,int track_id, int &obj_idx) 
{
    // Check if the object is old
    obj_idx = -1;
    for (int i = 0; i < objects_positions_.size(); i++) {
        PoseObjectPosition obj = objects_positions_[i];
        if (obj.track_id == track_id) {
            // Calculate Euclidean distance between detection centers
            // float dx = position.point.x - obj.pose.pose.position.x;
            // float dy = position.point.y - obj.pose.pose.position.y;
            // float dz = position.point.z - obj.pose.pose.position.z;

            // float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            // ROS_INFO("Distance: %f", distance);
            // // If the distance is below a threshold, consider it the same object

            // float distance_threshold = 0.1f;  // Pixel distance threshold, adjust as needed
            // if (distance < distance_threshold) {
                obj_idx = i;
                return true;
            // }
        }
        else {
            float dx = position.point.x - obj.position.x;
            float dy = position.point.y - obj.position.y;
            float dz = position.point.z - obj.position.z;

            float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
            ROS_INFO("Distance: %f", distance);

            float distance_threshold = 1.0f;  // Adjust threshold as needed
            if (distance < distance_threshold) {
                obj_idx = i;
                return true;
            }
        }
    }
    return false;
}

void RectbotObjectMapper::publishMarkers() {
    static int old_id = 0;
    visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.ns = "objects";
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.color.r = 0;
    m.color.g = 0;
    m.color.b = 255;
    m.color.a = 255;
     // lives forever
    m.lifetime = ros::Duration(0);
    m.frame_locked = true;
    int new_id = 0;
    // ROS_INFO("Object size: %d",objects_positions_.size());
    for (auto &obj : objects_positions_) {
        m.id = obj.track_id;
        m.type = visualization_msgs::Marker::CUBE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = obj.position.x;
        m.pose.position.y = obj.position.y;
        m.pose.position.z = obj.position.z;
        m.pose.orientation.x = 0;
        m.pose.orientation.y = 0;
        m.pose.orientation.z = 0;
        m.pose.orientation.w = 1;
        m.scale.x = 0.05;
        m.scale.y = 0.05;
        m.scale.z = 0.05;
        markers.push_back(m);
    }
    for (int i = new_id ; i < old_id; i++) {
        m.id = i;
        m.action = visualization_msgs::Marker::DELETE;
        markers.push_back(m);
    }
    object_marker_pub_.publish(markers_msg);
    old_id = new_id;
}

void RectbotObjectMapper::publishTransform()
{
    int object_id = 0;
    for (auto obj : objects_positions_) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = object_frame_id_ + std::to_string(object_id++);
        transformStamped.header.stamp = ros::Time::now();
        
        
    
        //publish object tf
        transformStamped.transform.translation.x = obj.position.x;
        transformStamped.transform.translation.y = obj.position.y;
        transformStamped.transform.translation.z = obj.position.z;
        transformStamped.transform.rotation.x = 0.0;    
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;
        object_broadcaster_.sendTransform(transformStamped);
    }
}

void RectbotObjectMapper::processD2CAligned() 
{
    // static int id = 0;
    if (!depth2color_aligned_image_msg_ || !depth_image_msg_ || detections_msg_.empty() || !depth_camera_model_.initialized() || !color_camera_model_.initialized() || !depth2color_aligned_camera_model_.initialized())
    {
        ROS_WARN_THROTTLE(1.0, "Skipping detection processing. Missing data.");
        return;
    }

    sensor_msgs::ImageConstPtr depthCopy, alignedD2CCopy;
    std::vector<rectbot_cv::PoseObject> detCopy;
    image_geometry::PinholeCameraModel depthModel, alignedModel;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        depthCopy = depth_image_msg_;
        alignedD2CCopy = depth2color_aligned_image_msg_ ;
        detCopy = detections_msg_;
        depthModel = depth_camera_model_;
        alignedModel = depth2color_aligned_camera_model_;
    }

    ROS_INFO("Detection message timestamp: %f", detections_msg_[0].detection.header.stamp.toSec());
    ROS_INFO("Depth2Color aligned image message timestamp: %f", depth2color_aligned_image_msg_->header.stamp.toSec());

    // Check timestamp difference between detection message and depth2color aligned image
    ros::Duration time_diff = depth2color_aligned_image_msg_->header.stamp - detections_msg_[0].detection.header.stamp;
    if (std::abs(time_diff.toSec()) > 0.1) { // Reject if time difference is greater than 0.1 seconds
        ROS_WARN("Timestamp difference too large: %f seconds. Skipping processing.", time_diff.toSec());
        return;
    }
    // m.header.frame_id = depth2color_aligned_image_msg_->header.frame_id;
    // visualization_msgs::MarkerArray markers_msg;
    // std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
    // visualization_msgs::Marker m;
    // m.header.frame_id = "camera_color_frame";
    // m.header.stamp = ros::Time::now();
    // m.ns = "objects";
    // m.scale.x = 1.0;
    // m.scale.y = 1.0;
    // m.scale.z = 1.0;
    // m.color.r = 0;
    // m.color.g = 0;
    // m.color.b = 255;
    // m.color.a = 255;
    //  // lives forever
    // m.lifetime = ros::Duration(0);
    // m.frame_locked = true;

    //get depth image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(alignedD2CCopy, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // std::vector<rectbot_cv::PoseObject> detections_copy = detections_msg_;
    cv::Mat depth2color_image = cv_ptr->image;
    // Now we can process the detections with the extrinsic matrix
    int new_id = 0;
    float center_point[3] = {0};


    //Bytetrack all detected object
    std::vector<bytetrack_cpp::Object> bytetrack_objects;
    for (auto det : detCopy) {
        bytetrack_cpp::Object obj;
        obj.label = det.detection.results[0].id;
        obj.rect = cv::Rect(det.detection.bbox.center.x - det.detection.bbox.size_x/2, det.detection.bbox.center.y - det.detection.bbox.size_y/2, det.detection.bbox.size_x, det.detection.bbox.size_y);
        obj.prob = det.detection.results[0].score;
        for (auto & kp : det.keypoints) {
            bytetrack_cpp::KeyPoint kpt;
            kpt.x = kp.x.data;
            kpt.y = kp.y.data;
            kpt.score = kp.score.data;
            obj.keypoints.push_back(kpt);
        }
        bytetrack_objects.push_back(obj);
    }

    const std::vector<bytetrack_cpp::STrack> output_track_objs = tracker_.update(bytetrack_objects);
    ROS_INFO("Detect objects: %ld, Output Tracker: %ld\r\n", bytetrack_objects.size(),
            output_track_objs.size());
    // objects_positions_.clear();

    for (auto obj : output_track_objs) {
        // int obj_idx = -1;

        // float depth_pixel[2] { 0.0f, 0.0f };
        // float min_distance = std::numeric_limits<float>::max();
        // for (const auto& keypoint : obj.keypoints) {
        //     if (keypoint.score > 0.7) {
        //     float distance = std::sqrt(std::pow(keypoint.x - obj.tlwh[0] - obj.tlwh[2] / 2, 2) +
        //                    std::pow(keypoint.y - obj.tlwh[1] - obj.tlwh[3] / 2, 2));
        //     if (distance < min_distance) {
        //         min_distance = distance;
        //         depth_pixel[0] = keypoint.x;
        //         depth_pixel[1] = keypoint.y;
        //     }
        //     }
        // }

        // // depth_pixel[2] = {obj.tlwh[0] + obj.tlwh[2]/2, obj.tlwh[1] + obj.tlwh[3]/2};
        // // float depth_pixel[2] = {}
        // // printf("Top: %f, Left: %f\r\n", obj.tlwh[1], obj.tlwh[0]);
        // // printf("Depth pixel: %f,%f\r\n",depth_pixel[0],depth_pixel[1]);
        // float depth_value = depth2color_image.at<uint16_t>(depth_pixel[1],depth_pixel[0]);
        // rs2_deproject_pixel_to_point(center_point, &color_intrinsics_ ,depth_pixel, depth_value * 0.001f);
        float min_depth = 3.0;
        float min_depth_pixel[2] = {0.0f, 0.0f};
        for (const auto& keypoint : obj.keypoints) {
                // printf("Keypoint[%ld]: %f,%f,%f\r\n", &keypoint - &obj.keypoints[0], keypoint.x, keypoint.y, keypoint.score);
            if (keypoint.score > 0.7) {
                float depth_pixel[2] = {keypoint.x, keypoint.y};
                float depth_value = depth2color_image.at<uint16_t>(depth_pixel[1], depth_pixel[0]) * 0.001f;
                if (depth_value <= 0.0f) continue;   // skip invalid depth
                // printf("Min depth: %f\r\n", min_depth);
                float point[3] = {0};
                if (depth_value < min_depth) {
                // printf("Keypoint[%ld]: %f,%f,%f\r\n", &keypoint - &obj.keypoints[0], keypoint.x, keypoint.y, depth_value);
                    min_depth = depth_value;
                    min_depth_pixel[0] = depth_pixel[0];
                    min_depth_pixel[1] = depth_pixel[1];
                    // printf("Depth value:\r\n");

                }
            }
        }
        if (min_depth >= 3) {
            continue;
        }
        if (min_depth_pixel[0] == 0.0f && min_depth_pixel[1] == 0.0f) {
            continue;
        }
        // printf("Depth value: %f\r\n", min_depth );
        // printf("Min depth pixel: %f,%f\r\n",min_depth_pixel[0],min_depth_pixel[1]);
        float depth_pixel[2] = {min_depth_pixel[0], min_depth_pixel[1]};
        rs2_deproject_pixel_to_point(center_point, &color_intrinsics_, depth_pixel, min_depth);
        // printf("Center point: %f,%f,%f\r\n",center_point[0],center_point[1],center_point[2]);
        if (center_point[2] < 0.1f || center_point[2] > 10.0f) {
            ROS_WARN("Invalid depth value: %f, skipping object", center_point[2]);
            continue;
        }
        // Transform the point
         geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(
                "map",           // Target frame
                depth2color_aligned_frame_id_,           // Source frame
                detCopy[0].detection.header.stamp// Get the latest transform
        );
        }
            catch (tf2::TransformException &ex) {
            ROS_WARN("Except: %s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::PointStamped point_in_cam_link;
        point_in_cam_link.header.frame_id = depth2color_aligned_frame_id_;
        point_in_cam_link.header.stamp = detCopy[0].detection.header.stamp;

        point_in_cam_link.point.x = center_point[2];
        point_in_cam_link.point.y = -center_point[0];
        point_in_cam_link.point.z = -center_point[1];
        geometry_msgs::PointStamped point_in_map;
        // float bbox_area_in_pix = det.detection.bbox.size_x * det.bbox.size_y;
        tf2::doTransform(point_in_cam_link, point_in_map, transform);
        point_in_map.header.frame_id = "map";
        // printf("Point in map: %f,%f,%f\r\n",point_in_map.point.x,point_in_map.point.y,point_in_map.point.z);
        int obj_idx = -1;
        if (isOldObject(point_in_map, obj.track_id, obj_idx)) {
            // Update the position of the existing object
            ROS_INFO("Old object detected: %d", obj.track_id);
            float dx = objects_positions_[obj_idx].position.x - point_in_map.point.x;
            float dy = objects_positions_[obj_idx].position.y - point_in_map.point.y;

            float distance = std::sqrt(dx * dx + dy * dy);

            if (distance < 0.2f || distance > 0.5f) {
                ROS_WARN("Distance exceeds 1 meter, skipping update for object ID: %d", objects_positions_[obj_idx].track_id);
                continue;
            }

            objects_positions_[obj_idx].position.x = 0.8 * objects_positions_[obj_idx].position.x + 0.2 * point_in_map.point.x;
            objects_positions_[obj_idx].position.y = 0.8 * objects_positions_[obj_idx].position.y + 0.2 * point_in_map.point.y;
            objects_positions_[obj_idx].position.z = 0.0;
            continue;
        }
        // Create a marker for the object
        // ROS_INFO("New object detected: %d", det.results[0].id);
        PoseObjectPosition new_obj;
        // new_obj.header = det.header;
        // new_obj.pose.header = det.header;
        new_obj.track_id = obj.track_id;
        new_obj.score = obj.score;
        new_obj.position.x = point_in_map.point.x;
        new_obj.position.y = point_in_map.point.y;
        new_obj.position.z = 0.0;
        // Get indices of keypoints with score above a threshold
        std::vector<int> high_score_keypoints_indices;
        float score_threshold = 0.7; // Adjust the threshold as needed
        for (size_t i = 0; i < obj.keypoints.size(); ++i) {
            if (obj.keypoints[i].score > score_threshold) {
            // high_score_keypoints_indices.push_back(i);
            if (i > 4) {
                if (i > 10) {
                    if (i >12) {
                        new_obj.body_part.leg = true;
                    }
                    else {
                        new_obj.body_part.body = true;
                    }
                }
                else {
                    new_obj.body_part.arm = true;
                }
            }
            else {
                new_obj.body_part.head = true;
            }
            }
        }
        objects_positions_.push_back(new_obj);
    }
    // for (const auto& obj : objects_positions_) {
    //     ROS_INFO("Object ID: %d", obj.track_id);
    //     ROS_INFO("Position: x=%.2f, y=%.2f, z=%.2f", obj.position.x, obj.position.y, obj.position.z);
    //     ROS_INFO("Body Parts:");
    //     if (obj.body_part.head) {
    //         ROS_INFO(" - Head");
    //     }
    //     if (obj.body_part.arm) {
    //         ROS_INFO(" - Arm");
    //     }
    //     if (obj.body_part.body) {
    //         ROS_INFO(" - Body");
    //     }
    //     if (obj.body_part.leg) {
    //         ROS_INFO(" - Leg");
    //     }
    // }
    publishMarkers();
    // publishTransform();

    // detCopy.clear(); // Clear the detections message after processing
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rectbot_object_mapping_node");
    RectbotObjectMapper mapper;
    ros::spin();
    return 0;
}