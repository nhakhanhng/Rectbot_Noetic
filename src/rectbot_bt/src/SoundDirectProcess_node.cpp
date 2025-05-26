#include "SoundDirectProcess_node.hpp"
#include <cmath>  




GetDirection::GetDirection(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config), nh_() {}

BT::NodeStatus GetDirection::tick()
{
  // std::string topic = getDirectionTopic();
  auto sound_msg = ros::topic::waitForMessage<std_msgs::Float32>('/sound_direction', nh_, ros::Duration(2.0));
  if (!sound_msg)
  {
    ROS_WARN("No direction received on: %s", topic.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto pose_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(
    "/odometr/filtered", nh_, ros::Duration(2.0));
  if (!pose_msg)
  {
    ROS_WARN("No pose received from /odometr/filtered");
    return BT::NodeStatus::FAILURE;
  }

  setOutput(directionPortName(), sound_msg->data);
  setOutput(posePortName(), pose_msg->pose.pose);

  return BT::NodeStatus::SUCCESS;
}



FindIntersectionPoint::FindIntersectionPoint(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList FindIntersectionPoint::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::Pose>("FirstRobotPose"),
    BT::InputPort<geometry_msgs::Pose>("SecondRobotPose"),
    BT::InputPort<double>("FirstDirection"),
    BT::InputPort<double>("SecondDirection"),
    BT::OutputPort<geometry_msgs::Pose>("InitSourcePose")
  };
}

BT::NodeStatus FindIntersectionPoint::tick()
{
  geometry_msgs::Pose pose1, pose2;
  double angle1_deg, angle2_deg;

  if (!getInput("FirstRobotPose", pose1) ||
      !getInput("SecondRobotPose", pose2) ||
      !getInput("FirstDirection", angle1_deg) ||
      !getInput("SecondDirection", angle2_deg))
  {
    ROS_ERROR("Missing required input");
    return BT::NodeStatus::FAILURE;
  }

  auto [dx1, dy1] = angleToUnitVector(angle1_deg);
  auto [dx2, dy2] = angleToUnitVector(angle2_deg);

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

auto [dx2, dy2] = angleToUnitVector(angle2_deg);
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

  setOutput("InitSourcePose", intersection);
  return BT::NodeStatus::SUCCESS;
}

std::pair<double, double> FindIntersectionPoint::angleToUnitVector(double angle_deg)
{
  double angle_rad =  angle_deg * M_PI / 180.0;
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

BT::NodeStatus FindNextPosition::tick()
{
  geometry_msgs::Pose current_pose;
  if (!getInput("CurrentPose", current_pose)) {
    ROS_ERROR("FindNextPosition: Missing CurrentPose");
    return BT::NodeStatus::FAILURE;
  }

  double yaw = tf::getYaw(current_pose.orientation);

  std::vector<double> angle_offsets_deg;
  for (int i = -90; i <= 90; i += 5) angle_offsets_deg.push_back(i);

  // Sort offsets by absolute value to prioritize minimal turn
  std::sort(angle_offsets_deg.begin(), angle_offsets_deg.end(),
            [](double a, double b) { return std::abs(a) < std::abs(b); });

  // Try FORWARD directions first
  for (double offset_deg : angle_offsets_deg) {
    double offset_rad = wrapToPi(offset_deg * M_PI / 180.0);
    double test_yaw = wrapToPi(yaw + offset_rad);

    double x = current_pose.position.x + test_move_distance_ * std::cos(test_yaw);
    double y = current_pose.position.y + test_move_distance_ * std::sin(test_yaw);

    if (isPoseSafe(x, y)) {
      ROS_INFO("Safe FORWARD direction found: %.1f°", test_yaw * 180 / M_PI);
      setOutput("NextPose", generatePose(x, y, test_yaw));
      return BT::NodeStatus::SUCCESS;
    }
  }

  // Try BACKWARD directions
  for (double offset_deg : angle_offsets_deg) {
    double offset_rad = wrapToPi(offset_deg * M_PI / 180.0);
    double test_yaw = wrapToPi(yaw + offset_rad + M_PI);  // Flip 180°

    double x = current_pose.position.x + test_move_distance_ * std::cos(test_yaw);
    double y = current_pose.position.y + test_move_distance_ * std::sin(test_yaw);

    if (isPoseSafe(x, y)) {
      ROS_INFO("Safe BACKWARD direction found: %.1f°", test_yaw * 180 / M_PI);
      setOutput("NextPose", generatePose(x, y, test_yaw));
      return BT::NodeStatus::SUCCESS;
    }
  }

  ROS_WARN("FindNextPosition: No safe direction found (forward or backward)");
  return BT::NodeStatus::FAILURE;
}

#include "your_package/GotoPos.hpp"
#include <ros/ros.h>
#include <tf/tf.h>

GotoPos::GotoPos(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
  ac_ = std::make_shared<MoveBaseClient>("move_base", true);

  ROS_INFO("GotoPos: Đợi move_base...");
  ac_->waitForServer();
  ROS_INFO("GotoPos: Kết nối thành công với move_base");
}

BT::PortsList GotoPos::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::Pose>("goal_pose")
  };
}

BT::NodeStatus GotoPos::tick()
{
  geometry_msgs::Pose goal_pose;
  if (!getInput("goal_pose", goal_pose)) {
    ROS_ERROR("GotoPos: miss input [goal_pose]");
    return BT::NodeStatus::FAILURE;
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = goal_pose;

  ROS_INFO("GotoPos:  (%.2f, %.2f)",
           goal_pose.position.x, goal_pose.position.y);

  ac_->sendGoal(goal);

  bool finished = ac_->waitForResult(ros::Duration(30.0));

  if (!finished)
  {
    ROS_WARN("GotoPos: time out move_base");
    ac_->cancelGoal();
    return BT::NodeStatus::FAILURE;
  }

  if (ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("GotoPos: Success");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    ROS_WARN("GotoPos: Fail - %s", ac_->getState().toString().c_str());
    return BT::NodeStatus::FAILURE;
  }
}
