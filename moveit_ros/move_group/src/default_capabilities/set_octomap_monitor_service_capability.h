/*
 * Author: Joshua Supratman
 * Description: service to enable/disable octomap monitor
 */
#ifndef MOVEIT_MOVE_GROUP_SET_OCTOMAP_MONITOR_SERVICE_CAPABILITY_
#define MOVEIT_MOVE_GROUP_SET_OCTOMAP_MONITOR_SERVICE_CAPABILITY_

#include <moveit/move_group/move_group_capability.h>
#include <std_srvs/SetBool.h>

namespace move_group
{
class SetOctomapMonitorService : public MoveGroupCapability
{
public:
  SetOctomapMonitorService();

  void initialize() override;

private:
  bool setOctomapMonitor(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  ros::ServiceServer service_;
};
}

#endif  // MOVEIT_MOVE_GROUP_SET_OCTOMAP_MONITOR_SERVICE_CAPABILITY_
