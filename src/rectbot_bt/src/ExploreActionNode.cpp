#include "ExploreActionNode.hpp"
#include <behaviortree_cpp_v3/behavior_tree.h>

ExploreActionNode::ExploreActionNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config), ac_("explore_action", true), goal_sent_(false), feedback_received_(false)
{
    ROS_INFO("ExploreActionNode initializing...");
    ac_.waitForServer();
    ROS_INFO("Connected to action server '/explore'.");
}

BT::PortsList ExploreActionNode::providedPorts()
{
    return {};  // No ports needed since ExplorationGoal is empty
}

BT::NodeStatus ExploreActionNode::tick()
{
    if (!goal_sent_) {
        ROS_INFO("Sending exploration goal...");
        new_explore::ExplorationGoal goal;
        ac_.sendGoal(goal,
                     boost::bind(&ExploreActionNode::doneCB, this, _1, _2),
                     actionlib::SimpleActionClient<new_explore::ExplorationAction>::SimpleActiveCallback(),
                     boost::bind(&ExploreActionNode::feedbackCB, this, _1));
        goal_sent_ = true;
    }

    // Return RUNNING if the action is still active
    if (ac_.getState().isDone()) {
        if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            new_explore::ExplorationResultConstPtr result = ac_.getResult();
            if (result->complete) {
                ROS_INFO("Exploration completed successfully.");
                goal_sent_ = false;  // Reset for next tick
                return BT::NodeStatus::SUCCESS;
            } else {
                ROS_ERROR("Exploration completed but result indicates failure.");
                goal_sent_ = false;
                return BT::NodeStatus::FAILURE;
            }
        } else if (ac_.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
            ROS_INFO("Exploration preempted.");
            goal_sent_ = false;
            return BT::NodeStatus::FAILURE;
        } else {
            ROS_ERROR("Exploration failed with state: %s", ac_.getState().toString().c_str());
            goal_sent_ = false;
            return BT::NodeStatus::FAILURE;
        }
    }

    // If feedback received, log it to confirm activity
    if (feedback_received_) {
        ROS_INFO("Exploration still running, feedback received.");
    } else {
        ROS_INFO("Exploration still running, waiting for feedback...");
    }

    return BT::NodeStatus::RUNNING;
}

void ExploreActionNode::halt()
{
    ROS_INFO("Halting exploration...");
    ac_.cancelGoal();
    goal_sent_ = false;
    feedback_received_ = false;
    setStatus(BT::NodeStatus::IDLE);
}

void ExploreActionNode::feedbackCB(const new_explore::ExplorationFeedbackConstPtr& feedback)
{
    feedback_received_ = true;
    ROS_INFO("Received feedback");
}

void ExploreActionNode::doneCB(const actionlib::SimpleClientGoalState& state,
                               const new_explore::ExplorationResultConstPtr& result)
{
    ROS_INFO("Goal finished with state: %s, complete: %d", state.toString().c_str(), result->complete);
}