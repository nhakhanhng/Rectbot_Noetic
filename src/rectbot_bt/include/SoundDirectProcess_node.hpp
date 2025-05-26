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


class GetDirection : public BT::SyncActionNode
{
public:
  GetDirection(const std::string& name, const BT::NodeConfiguration& config);

  virtual BT::NodeStatus tick() override;

protected:
  ros::NodeHandle nh_;

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
      BT::OutputPort<std::string>("FirstDirection"),
      BT::OutputPort<geometry_msgs::Pose>("FirstRobotPos")
    };
  }

protected:
  // std::string getDirectionTopic() const override { return "/sound_direction_first"; }
  std::string directionPortName() const override { return "FirstDirection"; }
  std::string posePortName() const override      { return "FirstRobotPose"; }
};



class GetSecondDirection : public GetDirection
{
public:
  GetSecondDirection(const std::string& name, const BT::NodeConfiguration& config)
    : GetDirection(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::string>("SecondDirection"),
      BT::OutputPort<geometry_msgs::Pose>("SecondRobotPos")
    };
  }

protected:
  // std::string getDirectionTopic() const override { return "/sound_direction_second"; }
  std::string directionPortName() const override { return "SecondDirection"; }
  std::string posePortName() const override      { return "SecondRobotPose"; }
};

class FindIntersectionPoint : public BT::SyncActionNode
{
public:
  FindIntersectionPoint(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::pair<double, double> angleToUnitVector(double angle_deg);
  bool computeLineIntersection(
    double x1, double y1, double dx1, double dy1,
    double x2, double y2, double dx2, double dy2,
    double& out_x, double& out_y);
};

#endif 