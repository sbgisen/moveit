
#include "clear_bbx_octomap_service_capability.h"
#include <moveit/move_group/capability_names.h>

move_group::ClearBBXOctomapService::ClearBBXOctomapService() : MoveGroupCapability("ClearBBXOctomapService")
{
}

void move_group::ClearBBXOctomapService::initialize()
{
  service_ = root_node_handle_.advertiseService(CLEAR_BBX_OCTOMAP_SERVICE_NAME,
                                                &ClearBBXOctomapService::clearBBXOctomap, this);
}

bool move_group::ClearBBXOctomapService::clearBBXOctomap(octomap_msgs::BoundingBoxQuery::Request& req,
                                                         octomap_msgs::BoundingBoxQuery::Response& /*resp*/)
{
  octomap::point3d min = octomap::pointMsgToOctomap(req.min);
  octomap::point3d max = octomap::pointMsgToOctomap(req.max);
  if (!context_->planning_scene_monitor_)
  {
    ROS_ERROR("Cannot clear bbx octomap since planning_scene_monitor_ does not exist.");
    return true;
  }

  ROS_INFO("Clearing BoundingBox from octomap.");
  context_->planning_scene_monitor_->clearBBXOctomap(min, max);
  ROS_INFO("BoundingBox cleared from octomap.");
  return true;
}

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::ClearBBXOctomapService, move_group::MoveGroupCapability)
