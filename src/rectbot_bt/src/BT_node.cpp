#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include "behaviortree_cpp_v3/action_node.h"

// Include your custom BT node headers
#include "PositionProcess_node.hpp"
#include "SoundDirectProcess_node.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "BT_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  BT::BehaviorTreeFactory factory;

  // Register all custom nodes
  factory.registerNodeType<GetFirstDirection>("GetFirstDirection");
  factory.registerNodeType<GetSecondDirection>("GetSecondDirection");
  factory.registerNodeType<FindIntersectionPoint>("FindIntersectionPoint");
  factory.registerNodeType<FindNextPosition>("FindNextPosition");
  factory.registerNodeType<GotoPos>("GotoPos");
  factory.registerNodeType<CancelAllGoal>("CancelAllGoal");
  factory.registerNodeType<CheckSoundNode>("CheckSound");
  factory.registerNodeType<StoreDirectionsPose>("StoreDirectionPose");
  factory.registerNodeType<GenerateCircularCandidates>("FindCandidatePos");
  factory.registerNodeType<GetNextCandidate>("GetNextCandidate");

  // Load XML file path from ROS param
  ROS_INFO("Loading Behavior Tree XML file from parameter: bt_xml_file");
  std::string xml_file;
  nh.param<std::string>("bt_xml_file", xml_file, "/home/arar/Documents/rectbot_ws/src/rectbot_bt/bt/rectbot_sound.xml");

  // Optional blackboard for shared data
  auto blackboard = BT::Blackboard::create();

  // Build tree from XML
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  // Logging
  BT::StdCoutLogger logger(tree);
  BT::PublisherZMQ publisher_zmq(tree);  // Optional: visualize via Groot

  ROS_INFO("Behavior Tree loaded: %s", xml_file.c_str());

  ros::Rate rate(10);
  while (ros::ok())
  {
    BT::NodeStatus status = tree.tickRoot();
    // if (status == BT::NodeStatus::SUCCESS)
    // {
    //     break;
    // }
    rate.sleep();
  }

  ROS_INFO("Behavior Tree execution completed.");
  return 0;
}
