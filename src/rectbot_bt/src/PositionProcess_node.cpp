#include "PositionProcess_node.hpp"
#include "SoundDirectProcess_node.hpp"
#include <tf/tf.h>
#include <cmath>

FindNextPosition::FindNextPosition(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
  costmap_sub_ = nh_.subscribe("/move_base/global_costmap/costmap", 1,
                               &FindNextPosition::costmapCallback, this);
}

BT::PortsList FindNextPosition::providedPorts()
{
  return {
    BT::InputPort<double>("FirstDirection"),
    BT::InputPort<geometry_msgs::Pose>("FirstRobotPos"),
    BT::OutputPort<geometry_msgs::Pose>("Position")
  };
}

void FindNextPosition::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  latest_costmap_ = *msg;
  costmap_received_ = true;
}

bool FindNextPosition::isPoseSafe(double x, double y)
{
  if (!costmap_received_)
  {
    ROS_WARN_THROTTLE(2.0, "Costmap not received yet");
    return false;
  }

  const auto& map = latest_costmap_;
  double map_origin_x = map.info.origin.position.x;
  double map_origin_y = map.info.origin.position.y;
  double resolution = map.info.resolution;
  int width = map.info.width;
  int height = map.info.height;

  int mx = (int)((x - map_origin_x) / resolution);
  int my = (int)((y - map_origin_y) / resolution);

  if (mx < 0 || my < 0 || mx >= width || my >= height)
  {
    ROS_DEBUG("Pose (%.2f, %.2f) out of map bounds", x, y);
    return false;
  }

  int index = my * width + mx;
  int cost = map.data[index];

  if (cost == 0) return true;       // Free
  if (cost == -1) return false;     // Unknown → considered unsafe
  return false;                     // Obstacle or lethal
}

BT::NodeStatus FindNextPosition::tick()
{
  double FirstDirection = 0;
  if (!getInput("FirstDirection", FirstDirection)) {
    ROS_ERROR("FindNextPosition: Missing FirstDirection");
    return BT::NodeStatus::FAILURE;
  }
  geometry_msgs::Pose current_pose;
  if (!getInput("FirstRobotPos", current_pose)) {
    ROS_ERROR("FindNextPosition: Missing CurrentPose");
    return BT::NodeStatus::FAILURE;
  }
  float del_angle = 0;
  if (FirstDirection < M_PI && FirstDirection > 0) {
    del_angle = -M_PI/2 + FirstDirection;
  } else {
    del_angle = M_PI/2 +  FirstDirection;
  }
  // del_angle = (del_angle * M_PI / 180.0);
  double yaw = tf::getYaw(current_pose.orientation);
  yaw = yaw + del_angle;
  std::vector<double> angle_offsets_deg;
  for (int i = -90; i <= 90; i += 5) angle_offsets_deg.push_back(i);

  // Sort offsets by absolute value to prioritize minimal turn
  std::sort(angle_offsets_deg.begin(), angle_offsets_deg.end(),
            [](double a, double b) { return std::abs(a) < std::abs(b); });

  // Try FORWARD directions first
  for (double offset_deg : angle_offsets_deg) {
    double offset_rad = (offset_deg * M_PI / 180.0);
    double test_yaw = (yaw + offset_rad);

    double x = current_pose.position.x + test_move_distance_ * std::cos(test_yaw);
    double y = current_pose.position.y + test_move_distance_ * std::sin(test_yaw);

    if (isPoseSafe(x, y)) {
      ROS_INFO("Safe FORWARD direction found: %.1f°", test_yaw * 180 / M_PI);
      geometry_msgs::Pose NextPosition = generatePose(x, y, test_yaw);
      setOutput("Position", NextPosition);
      return BT::NodeStatus::SUCCESS;
    }
  }

  // Try BACKWARD directions
  for (double offset_deg : angle_offsets_deg) {
    double offset_rad = (offset_deg * M_PI / 180.0);
    double test_yaw = (yaw + offset_rad + M_PI);  // Flip 180°

    double x = current_pose.position.x + test_move_distance_ * std::cos(test_yaw);
    double y = current_pose.position.y + test_move_distance_ * std::sin(test_yaw);

    if (isPoseSafe(x, y)) {
      ROS_INFO("Safe BACKWARD direction found: %.1f°", test_yaw * 180 / M_PI);
      setOutput("Position", generatePose(x, y, test_yaw));
      return BT::NodeStatus::SUCCESS;
    }
  }

  ROS_WARN("FindNextPosition: No safe direction found (forward or backward)");
  return BT::NodeStatus::FAILURE;
}

double FindNextPosition::wrapToPi(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

geometry_msgs::Pose FindNextPosition::generatePose(double x, double y, double yaw)
{
  geometry_msgs::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = 0;

  tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
  tf::quaternionTFToMsg(q, p.orientation);
  return p;
}



GotoPos::GotoPos(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
  ac_ = std::make_shared<MoveBaseClient>("move_base", true);

  ROS_INFO("GotoPos: Waiting for move_base...");
  ac_->waitForServer();
  ROS_INFO("GotoPos: Successfully connected to move_base");
}

BT::PortsList GotoPos::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::Pose>("Position")
  };
}

BT::NodeStatus GotoPos::tick()
{
  // return BT::NodeStatus::FAILURE;
  geometry_msgs::Pose goal_pose;
  if (!getInput("Position", goal_pose)) {
    ROS_ERROR("GotoPos: miss input [goal_pose]");
    return BT::NodeStatus::FAILURE;
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = goal_pose;

  ROS_INFO("GotoPos:  (%.2f, %.2f)",
           goal_pose.position.x, goal_pose.position.y);

  ac_->sendGoal(goal);

  bool finished = ac_->waitForResult(ros::Duration(30.0));
  // bool finished = true ;
  return BT::NodeStatus::SUCCESS;
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


CancelAllGoal::CancelAllGoal(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
  ac_ = std::make_shared<MoveBaseClient>("move_base", true);

  ROS_INFO("CancelAllGoal: Waiting for move_base server...");
  ac_->waitForServer();
  ROS_INFO("CancelAllGoal: Connected to move_base");
}

BT::PortsList CancelAllGoal::providedPorts()
{
  return {};  // No input or output ports
}

BT::NodeStatus CancelAllGoal::tick()
{
  // return BT::NodeStatus::SUCCESS;
  if (!ac_->isServerConnected())
  {
    ROS_ERROR("CancelAllGoal: move_base action server not connected");
    return BT::NodeStatus::FAILURE;
  }

  ROS_INFO("CancelAllGoal: Sending cancel all goals");
  ac_->cancelAllGoals();
  return BT::NodeStatus::SUCCESS;
}