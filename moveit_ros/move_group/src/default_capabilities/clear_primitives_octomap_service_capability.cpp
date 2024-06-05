
#include "clear_primitives_octomap_service_capability.h"

move_group::ClearPrimitivesOctomapService::ClearPrimitivesOctomapService()
  : MoveGroupCapability("ClearPrimitivesOctomapService")
{
}

void move_group::ClearPrimitivesOctomapService::initialize()
{
  service_ = root_node_handle_.advertiseService(CLEAR_PRIMITIVES_OCTOMAP_SERVICE_NAME,
                                                &ClearPrimitivesOctomapService::clearPrimitivesOctomap, this);
}

bool move_group::ClearPrimitivesOctomapService::clearPrimitivesOctomap(
    moveit_msgs::RemovePrimitivesFromOctomap::Request& req, moveit_msgs::RemovePrimitivesFromOctomap::Response& res)
{
  if (!context_->planning_scene_monitor_)
  {
    ROS_ERROR("Cannot clear bbx octomap since planning_scene_monitor_ does not exist.");
    return false;
  }
  if (req.primitives.size() != req.primitive_poses.size() || req.primitives.size() == 0)
  {
    ROS_ERROR("No primitives to clear from octomap.");
    return false;
  }
  ROS_INFO("Clearing Primitives from octomap.");
  context_->planning_scene_monitor_->clearPrimitivesOctomap(req.primitives, req.primitive_poses);
  ROS_INFO("Primitives  cleared from octomap.");
  return true;
}

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::ClearPrimitivesOctomapService, move_group::MoveGroupCapability)
