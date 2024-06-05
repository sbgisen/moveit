
#pragma once
#include <moveit/move_group/capability_names.h>
#include <moveit/move_group/move_group_capability.h>
#include <moveit_msgs/BoundingVolume.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/RemovePrimitivesFromOctomap.h>

namespace move_group
{
class ClearPrimitivesOctomapService : public MoveGroupCapability
{
public:
  ClearPrimitivesOctomapService();

  void initialize() override;

private:
  bool clearPrimitivesOctomap(moveit_msgs::RemovePrimitivesFromOctomap::Request& req,
                              moveit_msgs::RemovePrimitivesFromOctomap::Response& res);

  ros::ServiceServer service_;
};
}  // namespace move_group
