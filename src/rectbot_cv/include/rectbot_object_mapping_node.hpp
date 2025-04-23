#ifndef _RECTBOT_OBJECT_MAPPING_NODE_HPP_

#define _RECTBOT_OBJECT_MAPPING_NODE_HPP_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_geometry/pinhole_camera_model.h>
#include <vector>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2/utils.h>


#include <tf/transform_broadcaster.h>
#include <librealsense2/rs.hpp>
#include <librealsense2-gl/rs_processing_gl.hpp>

// Include the message type for your object detection
// Example: #include <vision_msgs/Detection2DArray.h>
// Adjust this according to what object detection system you're using
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection3DArray.h>

class RectbotObjectMapper
{
    public:
        RectbotObjectMapper();
        ~RectbotObjectMapper();
        void loadParameters();
        void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
        void depth2colorAlignedCallback(const sensor_msgs::ImageConstPtr& msg);
        void detectionCallback(const vision_msgs::Detection2DArrayConstPtr& msg);
        void depthImageInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
        void colorImageInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
        void depth2colorAlignedInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
        void processDetections();
        void processD2CAligned();
        void getExtrinsicMatrix(rs2_extrinsics &extrinsic_matrix,std::string param_name);
        void getIntrinsicMatrix(rs2_intrinsics &intrinsic_matrix,std::string param_name);
        void loadCalibrationMatrix();
        void logCalibrationMatrix();
        void publishMarkers();
        void publishTransform();
        bool isOldObject(geometry_msgs::PointStamped position,int label_id, int &obj_idx);


    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_{"~"};
        ros::Subscriber depth_image_sub_;
        ros::Subscriber depth2color_aligned_sub_;
        ros::Subscriber detection_sub_;
        ros::Subscriber depth_info_sub_;
        ros::Subscriber color_info_sub_;
        ros::Subscriber depth2color_aligned_info_sub_;

        ros::Publisher object_marker_pub_;
        image_geometry::PinholeCameraModel depth_camera_model_;
        image_geometry::PinholeCameraModel color_camera_model_;
        image_geometry::PinholeCameraModel depth2color_aligned_camera_model_;
        std::vector<vision_msgs::Detection2D> detections_msg_;
        sensor_msgs::ImageConstPtr depth_image_msg_;
        sensor_msgs::ImageConstPtr depth2color_aligned_image_msg_;
        sensor_msgs::CameraInfoConstPtr depth_info_;
        sensor_msgs::CameraInfoConstPtr color_info_;
        sensor_msgs::CameraInfoConstPtr depth2color_aligned_info_;

        //parameters
        std::string depth_frame_id_;
        std::string color_frame_id_;
        std::string depth2color_aligned_frame_id_;
        std::string base_frame_id_;
        std::string object_frame_id_;
        std::string color_image_topic_;
        std::string depth_image_topic_;
        std::string detection_topic_;
        std::string depth_info_topic_;
        std::string color_info_topic_;
        std::string object_marker_topic_;
        std::string depth2color_aligned_topic_;
        std::string depth2color_aligned_info_topic_;

        rs2_extrinsics color2depth_extrinsic_;
        rs2_extrinsics depth2color_extrinsic_;
        rs2_intrinsics depth_intrinsics_;
        rs2_intrinsics color_intrinsics_;
        tf::TransformBroadcaster object_broadcaster_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        std::vector<vision_msgs::ObjectHypothesisWithPose> objects_positions_;
        // std::vector<vision_msgs::Detection3D> objects_positions_3d_;
        int markers_id_;
};

#endif