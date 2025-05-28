#ifndef SOUND_DIRECT_PROCESS_HPP
#define SOUND_DIRECT_PROCESS_HPP


#include <iostream>
#include <chrono>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>

namespace BT {
template <> inline geometry_msgs::Pose convertFromString(StringView str)
{
     // Get vector of StringView
    auto string_views = BT::splitString(str, ';');

    // Convert each to std::string
    std::vector<std::string> parts;
    for (const auto& sv : string_views)
    {
        parts.emplace_back(sv.data(), sv.size());
    }

    if (parts.size() != 7)
    {
        throw BT::RuntimeError("Invalid input for Pose. Expected format: x;y;z;qx;qy;qz;qw");
    }

    geometry_msgs::Pose pose;
    pose.position.x = std::stod(parts[0]);
    pose.position.y = std::stod(parts[1]);
    pose.position.z = std::stod(parts[2]);
    pose.orientation.x = std::stod(parts[3]);
    pose.orientation.y = std::stod(parts[4]);
    pose.orientation.z = std::stod(parts[5]);
    pose.orientation.w = std::stod(parts[6]);

    return pose;
}
}


class GetDirection : public BT::SyncActionNode
{
public:
  GetDirection(const std::string& name, const BT::NodeConfiguration& config);

  virtual BT::NodeStatus tick() override;
  void publishBearingMarker(double bearing_rad, const geometry_msgs::Pose& pose, int marker_id);


protected:
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;

  // These must be overridden by subclasses
  // virtual std::string getDirectionTopic() const = 0;
  virtual std::string directionPortName() const = 0;
  virtual std::string posPortName() const = 0;
};

class GetFirstDirection : public GetDirection
{
public:
  GetFirstDirection(const std::string& name, const BT::NodeConfiguration& config)
    : GetDirection(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<double>("FirstDirection"),
      BT::OutputPort<geometry_msgs::Pose>("FirstRobotPos")
    };
  }

protected:
  // std::string getDirectionTopic() const override { return "/sound_direction_first"; }
  std::string directionPortName() const override { return "FirstDirection"; }
  std::string posPortName() const override      { return "FirstRobotPos"; }
};



class GetSecondDirection : public GetDirection
{
public:
  GetSecondDirection(const std::string& name, const BT::NodeConfiguration& config)
    : GetDirection(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<double>("SecondDirection"),
      BT::OutputPort<geometry_msgs::Pose>("SecondRobotPos")
    };
  }

protected:
  // std::string getDirectionTopic() const override { return "/sound_direction_second"; }
  std::string directionPortName() const override { return "SecondDirection"; }
  std::string posPortName() const override      { return "SecondRobotPos"; }
};

class FindIntersectionPoint : public BT::SyncActionNode
{
public:
  FindIntersectionPoint(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
  void publishIntersectionPoint(const geometry_msgs::Pose& intersection);

private:
  std::pair<double, double> angleToUnitVector(double angle_deg);
  ros::NodeHandle nh_;
  ros::Publisher point_pub_;
  bool computeLineIntersection(
    double x1, double y1, double dx1, double dy1,
    double x2, double y2, double dx2, double dy2,
    double& out_x, double& out_y);
};

class CheckSoundNode : public BT::ConditionNode
{
public:
  CheckSoundNode(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  void soundCallback(const std_msgs::Float32::ConstPtr& msg);
  double angleDifference(double angle1, double angle2);

  ros::NodeHandle nh_;
  ros::Subscriber sound_sub_;
  int positive_count_;
  bool received_;
  float last_direct_;
  float current_direct_;
  ros::Time last_update_time_;

  const int required_count_ = 2;
  const double timeout_sec_ = 5.0;
};


#endif 