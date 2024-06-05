
#pragma once

#include <moveit/move_group/move_group_capability.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_ros/conversions.h>

namespace move_group
{
class ClearBBXOctomapService : public MoveGroupCapability
{
public:
  ClearBBXOctomapService();

  void initialize() override;

private:
  bool clearBBXOctomap(octomap_msgs::BoundingBoxQuery::Request& req, octomap_msgs::BoundingBoxQuery::Response& resp);

  ros::ServiceServer service_;
};
}  // namespace move_group
