#ifndef POSITION_PROCESS_NODE_HPP
#define POSITION_PROCESS_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

class FindNextPosition : public BT::SyncActionNode
{
public:
  FindNextPosition(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  bool isPoseSafe(double x, double y);
  double wrapToPi(double angle);
  geometry_msgs::Pose generatePose(double x, double y, double yaw);

  // Costmap
  void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  nav_msgs::OccupancyGrid latest_costmap_;
  bool costmap_received_ = false;
  ros::NodeHandle nh_;
  ros::Subscriber costmap_sub_;

  double test_move_distance_ = 1.0;  // meters
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


#endif
