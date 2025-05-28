#include "SoundDirectProcess_node.hpp"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <cmath>  




GetDirection::GetDirection(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config), nh_() {
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/sound_bearing_marker", 10);
  }

BT::NodeStatus GetDirection::tick()
{
  // std::string topic = getDirectionTopic();
  // return BT::NodeStatus::SUCCESS; // TESTING TREE
  auto sound_msg = ros::topic::waitForMessage<std_msgs::Float32>("/sound_direction", nh_, ros::Duration(2.0));
  if (!sound_msg)
  {
    ROS_WARN("No direction received on: %s", "/sound_direction");
    return BT::NodeStatus::FAILURE;
  }
  double angle = sound_msg->data;

  ROS_INFO("Received sound direction: %.2f degrees", angle);
  auto pose_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(
    "/odometry/filtered", nh_, ros::Duration(2.0));
  if (!pose_msg)
  {
    ROS_WARN("No pose received from /odometry/filtered");
    return BT::NodeStatus::FAILURE;
  }
  double robot_yaw = tf::getYaw(pose_msg->pose.pose.orientation);
  double odom_angle =  angle * M_PI / 180.0 + robot_yaw;
  if (directionPortName() == "FirstDirection")
  {
  publishBearingMarker(odom_angle, pose_msg->pose.pose, 1);
  }
  else 
  {
  publishBearingMarker(odom_angle, pose_msg->pose.pose, 2);
  }
  ROS_INFO("Calculated odom_angle: %.2f radians", odom_angle);
  setOutput(directionPortName(), odom_angle);
  setOutput(posPortName(), pose_msg->pose.pose);

  return BT::NodeStatus::SUCCESS;
}

void GetDirection::publishBearingMarker(double bearing_rad, const geometry_msgs::Pose& pose, int marker_id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "odom";
  marker.header.stamp = ros::Time::now();
  marker.ns = "bearing_marker";
  marker.id = marker_id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.05;  // Line width
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  geometry_msgs::Point start_point = pose.position;
  geometry_msgs::Point end_point;
  end_point.x = start_point.x + 2.0 * cos(bearing_rad);
  end_point.y = start_point.y + 2.0 * sin(bearing_rad);
  end_point.z = start_point.z;

  marker.points.push_back(start_point);
  marker.points.push_back(end_point);

  ROS_INFO("Publishing bearing marker: id=%d, start=(%.2f, %.2f), end=(%.2f, %.2f)",
           marker_id, start_point.x, start_point.y, end_point.x, end_point.y);

  marker_pub_.publish(marker);
}



FindIntersectionPoint::FindIntersectionPoint(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {
    point_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/intersection_point", 10);
  }

BT::PortsList FindIntersectionPoint::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::Pose>("FirstRobotPos"),
    BT::InputPort<geometry_msgs::Pose>("SecondRobotPos"),
    BT::InputPort<double>("FirstDirection"),
    BT::InputPort<double>("SecondDirection"),
    BT::OutputPort<geometry_msgs::Pose>("InitSourcePos")
  };
}

BT::NodeStatus FindIntersectionPoint::tick()
{
  geometry_msgs::Pose pose1, pose2;
  double angle1, angle2;
  // return BT::NodeStatus::SUCCESS; //testing

  if (!getInput("FirstRobotPos", pose1))
  {
    ROS_ERROR("Missing required input: FirstRobotPose");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("SecondRobotPos", pose2))
  {
    ROS_ERROR("Missing required input: SecondRobotPose");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("FirstDirection", angle1))
  {
    ROS_ERROR("Missing required input: FirstDirection");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("SecondDirection", angle2))
  {
    ROS_ERROR("Missing required input: SecondDirection");
    return BT::NodeStatus::FAILURE;
  }

  std::pair<double, double> dir1 = angleToUnitVector(angle1);
  double dx1 = dir1.first;
  double dy1 = dir1.second;
  std::pair<double, double> dir2 = angleToUnitVector(angle2);
  double dx2 = dir2.first;
  double dy2 = dir2.second;

  double ix, iy;
  bool success = computeLineIntersection(
    pose1.position.x, pose1.position.y, dx1, dy1,
    pose2.position.x, pose2.position.y, dx2, dy2,
    ix, iy);

   if (!success)
  {
    ROS_WARN("Lines are parallel or do not intersect");
    return BT::NodeStatus::FAILURE;
  }

  // Check distance
double dist = std::hypot(ix - pose2.position.x, iy - pose2.position.y);
if (dist > 4.0)
{
  ROS_WARN("Intersection too far (%.2f m)", dist);
  return BT::NodeStatus::FAILURE;
}

// Check angle (dot product test)
double vx = ix - pose2.position.x;
double vy = iy - pose2.position.y;
double v_norm = std::hypot(vx, vy);
if (v_norm < 1e-6) return BT::NodeStatus::FAILURE;
vx /= v_norm; vy /= v_norm;

 dir2 = angleToUnitVector(angle2);
 dx2 = dir2.first;
 dy2 = dir2.second;
double dot = vx * dx2 + vy * dy2;

if (dot < 0.0) // behind
{
  ROS_WARN("Intersection behind robot 2's bearing");
  return BT::NodeStatus::FAILURE;
}

  geometry_msgs::Pose intersection;
  intersection.position.x = ix;
  intersection.position.y = iy;
  intersection.position.z = 0;
  intersection.orientation.w = 1.0;  // identity orientation
  publishIntersectionPoint(intersection);
  setOutput("InitSourcePos", intersection);
  return BT::NodeStatus::SUCCESS;
}

void FindIntersectionPoint::publishIntersectionPoint(const geometry_msgs::Pose& intersection)
{
  geometry_msgs::PoseStamped intersection_msg;
  intersection_msg.header.frame_id = "odom";
  intersection_msg.header.stamp = ros::Time::now();
  intersection_msg.pose = intersection;

  ROS_INFO("Publishing intersection point: (%.2f, %.2f)", 
           intersection.position.x, intersection.position.y);

  point_pub_.publish(intersection_msg);
}

std::pair<double, double> FindIntersectionPoint::angleToUnitVector(double angle_rad)
{
  // double angle_rad =  angle_deg * M_PI / 180.0;
  return { cos(angle_rad), sin(angle_rad) };
}

bool FindIntersectionPoint::computeLineIntersection(
  double x1, double y1, double dx1, double dy1,
  double x2, double y2, double dx2, double dy2,
  double& out_x, double& out_y)
{
  double det = dx1 * dy2 - dy1 * dx2;
  if (fabs(det) < 1e-6)
    return false;  // parallel lines

  double t = ((x2 - x1) * dy2 - (y2 - y1) * dx2) / det;
  out_x = x1 + t * dx1;
  out_y = y1 + t * dy1;
  return true;
}

CheckSoundNode::CheckSoundNode(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config), positive_count_(0), received_(false), last_direct_(0), current_direct_(0)
{
  sound_sub_ = nh_.subscribe("/sound_direction", 1, &CheckSoundNode::soundCallback, this);
}

BT::PortsList CheckSoundNode::providedPorts()
{
  return {};
}

void CheckSoundNode::soundCallback(const std_msgs::Float32::ConstPtr& msg)
{
  current_direct_ = msg->data;
  received_ = true;
  last_update_time_ = ros::Time::now();
  ROS_INFO("[CheckSoundNode] Received sound direction: %.2f degrees", current_direct_);
  float angle_diff = angleDifference(current_direct_, last_direct_);
  ROS_INFO("[CheckSoundNode] Angle difference: %.2f degrees", angle_diff);
  if (angle_diff < 25.0)  // within 10 degrees
    positive_count_++;
  else
    positive_count_ = 0;  // reset if a single "false" is received
  last_direct_ = current_direct_;
}

double CheckSoundNode::angleDifference(double angle1, double angle2)
{
  double diff = angle1 - angle2;
  while (diff > 180.0) diff -= 360.0;
  while (diff < -180.0) diff += 360.0;
  return abs(diff);
}

BT::NodeStatus CheckSoundNode::tick()
{
  // ros::Duration(10.0).sleep();
  if (!received_)
  {
    ROS_WARN_THROTTLE(5.0, "[CheckSoundNode] No message received yet");
    return BT::NodeStatus::FAILURE;
  }

  // Reset if timeout
  if ((ros::Time::now() - last_update_time_).toSec() > timeout_sec_)
  {
    ROS_WARN_THROTTLE(2.0, "[CheckSoundNode] Timeout since last message");
    positive_count_ = 0;
    return BT::NodeStatus::FAILURE;
  }

  if (positive_count_ >= required_count_)
  {
    ROS_INFO("[CheckSoundNode] Sound detected %d times, returning SUCCESS", positive_count_);
    positive_count_ = 0;  // reset after success
    return BT::NodeStatus::SUCCESS;
  }

  ROS_INFO_THROTTLE(1.0, "[CheckSoundNode] Detected %d / %d times", positive_count_, required_count_);
  return BT::NodeStatus::FAILURE;
}
