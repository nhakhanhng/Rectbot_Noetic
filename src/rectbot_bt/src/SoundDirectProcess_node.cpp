#include "SoundDirectProcess_node.hpp"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <cmath>  




GetDirection::GetDirection(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config), nh_(), tf_listener_(nh_) {
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/sound_bearing_marker", 10);
  }

BT::NodeStatus GetDirection::tick()
{
  // std::string topic = getDirectionTopic();
  // return BT::NodeStatus::SUCCESS; // TESTING TREE
  auto sound_msg = ros::topic::waitForMessage<std_msgs::Float32>("/sound_direction", nh_, ros::Duration(5.0));
  if (!sound_msg)
  {
    ROS_WARN("No direction received on: %s", "/sound_direction");
    return BT::NodeStatus::FAILURE;
  }
  double angle = sound_msg->data;

  ROS_INFO("Received sound direction: %.2f degrees", angle);
  auto pose_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(
    "/odometry/filtered", nh_, ros::Duration(3.0));
  if (!pose_msg)
  {
    ROS_WARN("No pose received from /odometry/filtered");
    return BT::NodeStatus::FAILURE;
  }
  // Transform pose to map frame
  geometry_msgs::PoseStamped pose_in, pose_out;
  pose_in.header = pose_msg->header;
  pose_in.pose = pose_msg->pose.pose;

  try
  {
    
    tf_listener_.waitForTransform("map", pose_in.header.frame_id, pose_msg->header.stamp, ros::Duration(2.0));
    // ROS_INFO("Waiting for transform from %s to map", pose_in.header.frame_id.c_str());
    ROS_INFO("Wait complete");
    tf_listener_.transformPose("map", pose_in, pose_out);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("Transform error: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }
  double robot_yaw = tf::getYaw(pose_out.pose.orientation);
  double map_angle =  angle * M_PI / 180.0 + robot_yaw;
  if (directionPortName() == "FirstDirection")
  {
  publishBearingMarker(map_angle, pose_msg->pose.pose, 1);
  }
  else 
  {
  publishBearingMarker(map_angle, pose_msg->pose.pose, 2);
  }
  ROS_INFO("Calculated map_angle: %.2f radians", map_angle);
  setOutput(directionPortName(), map_angle);
  setOutput(posPortName(), pose_msg->pose.pose);

  return BT::NodeStatus::SUCCESS;
}

void GetDirection::publishBearingMarker(double bearing_rad, const geometry_msgs::Pose& pose, int marker_id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
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
    // map_sub_ = nh_.subscribe("/map", 1, &FindIntersectionPoint::mapCallback, this);
    // Service client for plan checking
    make_plan_client_ = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    make_plan_client_.waitForExistence(ros::Duration(5.0));
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


bool FindIntersectionPoint::isPlanValid(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal)
{
  nav_msgs::GetPlan srv;
  srv.request.start.header.frame_id = "map";
  srv.request.start.header.stamp = ros::Time::now();
  srv.request.start.pose = start;

  srv.request.goal.header.frame_id = "map";
  srv.request.goal.header.stamp = ros::Time::now();
  srv.request.goal.pose = goal;

  srv.request.tolerance = 0.5;  // 0.5m tolerance

  if (!make_plan_client_.call(srv)) {
    ROS_WARN("Failed to call make_plan service");
    return false;
  }
  return !srv.response.plan.poses.empty();
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
  // Check plan feasibility
  geometry_msgs::Pose goal_pose;
  goal_pose.position.x = ix;
  goal_pose.position.y = iy;
  goal_pose.position.z = 0;
  goal_pose.orientation.w = 1.0;
   if (!success)
  {
    ROS_WARN("Lines are parallel or do not intersect");
    return BT::NodeStatus::FAILURE;
  }

   if (!isPlanValid(pose2, goal_pose)) {
    ROS_WARN("No valid plan to intersection within 0.5m tolerance");
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
  intersection_msg.header.frame_id = "map";
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
  received_ = true;
  last_update_time_ = ros::Time::now();
  // ROS_INFO("[CheckSoundNode] Received sound direction: %.2f degrees", current_direct_);
  float angle_diff = angleDifference(current_direct_, last_direct_);
  // ROS_INFO("[CheckSoundNode] Angle difference: %.2f degrees", angle_diff);
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



StoreDirectionsPose::StoreDirectionsPose(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config), nh_(), tf_listener_(nh_)
{
  if (!ros::isInitialized())
  {
    int argc = 0; char** argv = nullptr;
    ros::init(argc, argv, "store_directions_pose_bt_node");
  }
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/sound_direction_marker", 10);
}

BT::PortsList StoreDirectionsPose::providedPorts()
{
  return {
    BT::OutputPort<std::vector<double>>("DirectionList"),
    BT::OutputPort<std::vector<geometry_msgs::Pose>>("PosList"),
    BT::InputPort<geometry_msgs::Pose>("InitPos")
  };
}

BT::NodeStatus StoreDirectionsPose::tick()
{

  geometry_msgs::Pose init_position;

  if (!getInput("InitPos", init_position))
  {
    ROS_WARN("Missing required input: InitPos");
    return BT::NodeStatus::FAILURE;
  }
  auto sound_msg = ros::topic::waitForMessage<std_msgs::Float32>("/sound_direction", nh_, ros::Duration(5.0));
  if (!sound_msg)
  {
    ROS_WARN("Timeout waiting for /sound_direction");
    return BT::NodeStatus::FAILURE;
  }

  auto odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>("/odometry/filtered", nh_, ros::Duration(2.0));
  if (!odom_msg)
  {
    ROS_WARN("Timeout waiting for /odom");
    return BT::NodeStatus::FAILURE;
  }



  double direction = sound_msg->data;
  geometry_msgs::Pose current_pose = odom_msg->pose.pose;
  // Transform current_pose to map frame
  geometry_msgs::PoseStamped pose_in, pose_out;
  pose_in.header = odom_msg->header;
  pose_in.pose = current_pose;
  

  try
  {
    tf_listener_.waitForTransform("map", pose_in.header.frame_id, odom_msg->header.stamp, ros::Duration(2.0));
    tf_listener_.transformPose("map", pose_in, pose_out);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("Transform error: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  // Calculate orientation from robot to init_position
  double dx = init_position.position.x - pose_out.pose.position.x;
  double dy = init_position.position.y - pose_out.pose.position.y;
  double orientation_to_init = std::atan2(dy, dx);

  // Normalize orientation to the range [-pi, pi]
  while (orientation_to_init > M_PI) orientation_to_init -= 2 * M_PI;
  while (orientation_to_init < -M_PI) orientation_to_init += 2 * M_PI;

  ROS_INFO("Orientation from robot to init_position: %.2f radians", orientation_to_init);

  // Compare sound direction with orientation to init_position
  double angle_diff = std::fabs(direction * M_PI / 180.0 - orientation_to_init);
  while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
  angle_diff = std::fabs(angle_diff);

  ROS_INFO("Angle difference between sound direction and orientation to init_position: %.2f radians", angle_diff);

  if (angle_diff > M_PI)  // Example threshold: 45 degrees
  {
    ROS_WARN("Sound direction deviates significantly from orientation to init_position");
    return BT::NodeStatus::FAILURE;
  }

  current_pose = pose_out.pose;
  double robot_yaw = tf::getYaw(current_pose.orientation);
  double map_angle =  direction * M_PI / 180.0 + robot_yaw;

  // Thêm dữ liệu mới (direction là 1 góc duy nhất, nên đóng gói thành vector)
  direction_list_.push_back(map_angle);
  pose_list_.push_back(current_pose);
  publishSoundDirectionMarker(map_angle, current_pose, direction_list_.size());

  // Cập nhật lại Blackboard
  setOutput("DirectionList", direction_list_);
  setOutput("PosList", pose_list_);

  return BT::NodeStatus::SUCCESS;
}

void StoreDirectionsPose::publishSoundDirectionMarker(double direction, const geometry_msgs::Pose& pose, int marker_id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "sound_direction_marker";
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
  end_point.x = start_point.x + 2.0 * cos(direction);
  end_point.y = start_point.y + 2.0 * sin(direction);
  end_point.z = start_point.z;

  marker.points.push_back(start_point);
  marker.points.push_back(end_point);

  ROS_INFO("Publishing sound direction marker: id=%d, start=(%.2f, %.2f), end=(%.2f, %.2f)",
           marker_id, start_point.x, start_point.y, end_point.x, end_point.y);

  marker_pub_.publish(marker);
}

FindSoundSource::FindSoundSource(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config), nh_()
{
  source_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/estimate_sound_source_pos", 10);
}

BT::PortsList FindSoundSource::providedPorts()
{
  return {
    BT::InputPort<std::vector<double>>("DirectionList"),
    BT::InputPort<std::vector<geometry_msgs::Pose>>("PosList"),
    BT::OutputPort<geometry_msgs::Pose>("SourcePos"),
    BT::OutputPort<bool>("isFound")
  };
}

void FindSoundSource::publishSourceMarker(const geometry_msgs::Pose& source_pos)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "source_marker";
  marker.id = 0;  // Single marker for the source
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = source_pos;
  marker.scale.x = 0.5;  // Sphere diameter
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  ROS_INFO("Publishing source marker at (%.2f, %.2f)", 
           source_pos.position.x, source_pos.position.y);

  source_marker_pub_.publish(marker);
}

BT::NodeStatus FindSoundSource::tick()
{
  std::vector<double> directions_list;
  std::vector<geometry_msgs::Pose> poses;

  if (!getInput("DirectionList", directions_list))
  {
    ROS_ERROR("Missing required input: DirectionList");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("PosList", poses))
  {
    ROS_ERROR("Missing required input: PosList");
    return BT::NodeStatus::FAILURE;
  }

  if (directions_list.size() != poses.size() || directions_list.size() < 2)
  {
    ROS_WARN("Input lists size mismatch or not enough data");
    return BT::NodeStatus::FAILURE;
  }

  std::vector<std::pair<double,double>> intersections;

  // Tính tất cả giao điểm cắt nhau của cặp direction từ các vị trí khác nhau
  for (size_t i = 0; i < directions_list.size(); ++i)
  {
    for (size_t j = i + 1; j < directions_list.size(); ++j)
    {
      // Giả sử directions_list[i] và directions_list[j] chỉ chứa 1 góc duy nhất
      double angle_i = directions_list[i];
      double angle_j = directions_list[j];

      std::pair<double,double> di = angleToUnitVector(angle_i);
      double dx_i = di.first;
      double dy_i = di.second;
      std::pair<double,double> dj = angleToUnitVector(angle_j);
      double dx_j = dj.first;
      double dy_j = dj.second;

      double ix, iy;
      bool success = computeLineIntersection(
        poses[i].position.x, poses[i].position.y, dx_i, dy_i,
        poses[j].position.x, poses[j].position.y, dx_j, dy_j,
        ix, iy);

      if (success)
      {
        intersections.emplace_back(ix, iy);
      }
    }
  }

  if (intersections.empty())
  {
    ROS_WARN("No valid intersections found");
    return BT::NodeStatus::FAILURE;
  }

  // Tính trung bình giao điểm (đơn giản nhất)
  double sum_x = 0, sum_y = 0;
  for (auto& p : intersections)
  {
    sum_x += p.first;
    sum_y += p.second;
  }

  double avg_x = sum_x / intersections.size();
  double avg_y = sum_y / intersections.size();

  geometry_msgs::Pose source_pos;
  source_pos.position.x = avg_x;
  source_pos.position.y = avg_y;
  source_pos.position.z = 0.0;
  source_pos.orientation.w = 1.0;  // identity quaternion

  // Publish the source marker
  publishSourceMarker(source_pos);

  setOutput("isFound", true);  // Indicate that the source was found
  setOutput("SourcePos", source_pos);

  return BT::NodeStatus::SUCCESS;
}

IsNotFoundYet::IsNotFoundYet(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{}

BT::PortsList IsNotFoundYet::providedPorts()
{
    return { BT::InputPort<bool>("isFound","false") };
}

BT::NodeStatus IsNotFoundYet::tick()
{
    bool is_found;
    if (!getInput("isFound", is_found))
    {
        throw BT::RuntimeError("Missing required input port: isFound");
    }

    return is_found ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}


std::pair<double,double> FindSoundSource::angleToUnitVector(double angle_rad)
{
  return { std::cos(angle_rad), std::sin(angle_rad) };
}

bool FindSoundSource::computeLineIntersection(
  double x1, double y1, double dx1, double dy1,
  double x2, double y2, double dx2, double dy2,
  double& out_x, double& out_y)
{
  double det = dx1*dy2 - dy1*dx2;
  if (std::fabs(det) < 1e-6) return false;
  double t = ((x2 - x1)*dy2 - (y2 - y1)*dx2) / det;
  out_x = x1 + t*dx1;
  out_y = y1 + t*dy1;
  return true;
}