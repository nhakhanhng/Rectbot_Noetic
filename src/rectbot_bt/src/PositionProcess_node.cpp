#include "PositionProcess_node.hpp"
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
    BT::InputPort<geometry_msgs::Pose>("CurrentPose"),
    BT::OutputPort<geometry_msgs::Pose>("NextPose")
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
