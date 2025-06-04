#ifndef EXPLORE_ACTION_NODE_H
#define EXPLORE_ACTION_NODE_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <new_explore/ExplorationAction.h>
#include <behaviortree_cpp_v3/action_node.h>

class ExploreActionNode : public BT::AsyncActionNode
{
public:
    ExploreActionNode(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<new_explore::ExplorationAction> ac_;
    bool goal_sent_;
    bool feedback_received_;

    void feedbackCB(const new_explore::ExplorationFeedbackConstPtr& feedback);
    void doneCB(const actionlib::SimpleClientGoalState& state,
                const new_explore::ExplorationResultConstPtr& result);
};

#endif  // EXPLORE_ACTION_NODE_H