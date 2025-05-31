#ifndef POSITION_PROCESS_NODE_HPP
#define POSITION_PROCESS_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <nav_msgs/GetPlan.h>

class FindNextPosition : public BT::SyncActionNode
{
public:
  FindNextPosition(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  bool isPoseSafe(double x, double y);
  double wrapToPi(double angle);
  double degreeToRadian(double degree) const { return degree * M_PI / 180.0; }
  geometry_msgs::Pose generatePose(double x, double y, double yaw);

  // Costmap
  void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  nav_msgs::OccupancyGrid latest_costmap_;
  bool costmap_received_ = false;
  ros::NodeHandle nh_;
  ros::Subscriber costmap_sub_;

  double test_move_distance_ = 0.5;  // meters
};

class GotoPos : public BT::SyncActionNode
{
public:
  GotoPos(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
  std::shared_ptr<MoveBaseClient> ac_;
};

class CancelAllGoal : public BT::SyncActionNode
{
public:
  CancelAllGoal(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
  std::shared_ptr<MoveBaseClient> ac_;
};

class GenerateCircularCandidates : public BT::SyncActionNode
{
public:
  GenerateCircularCandidates(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
  void publishCandidates(const std::vector<geometry_msgs::Pose>& candidates);

private:
  bool isPlanValid(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal);

  ros::NodeHandle nh_;
  ros::ServiceClient make_plan_client_;
  ros::Publisher marker_pub_;
};  


class GetNextCandidate : public BT::SyncActionNode
{
public:
  GetNextCandidate(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  size_t current_index_ = 0;  // theo dõi vị trí candidate hiện tại
};

#endif
