/*
 * Author: Joshua Supratman
 * Description: service to enable/disable octomap monitor
 */
#include "set_octomap_monitor_service_capability.h"
#include <moveit/move_group/capability_names.h>

move_group::SetOctomapMonitorService::SetOctomapMonitorService() : MoveGroupCapability("SetOctomapMonitorService")
{
}

void move_group::SetOctomapMonitorService::initialize()
{
  service_ = root_node_handle_.advertiseService(SET_OCTOMAP_MONITOR_SERVICE_NAME, &SetOctomapMonitorService::setOctomapMonitor, this);
}

bool move_group::SetOctomapMonitorService::setOctomapMonitor(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  if (!context_->planning_scene_monitor_)
  {
    ROS_ERROR("Cannot set octomap monitor since planning_scene_monitor_ does not exist.");
    res.success = false;
    return true;
  }
  if(req.data)
  {
    context_->planning_scene_monitor_->startOctomapMonitor();
    //context_->planning_scene_monitor_->clearOctomap();
    ROS_INFO("start octomap update");
  }
  else
  {
    context_->planning_scene_monitor_->stopOctomapMonitor();
    //context_->planning_scene_monitor_->clearOctomap();
    ROS_INFO("stop octomap update");
  }
  res.success = true;

  return true;
}

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::SetOctomapMonitorService, move_group::MoveGroupCapability)
