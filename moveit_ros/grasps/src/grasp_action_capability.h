/*
 * Author: Joshua Supratman
 * Description: moveit plugin (MoveGroupCapability) for using moveit_grasps
 */

#ifndef MOVEIT_MOVE_GROUP_GRASP_ACTION_CAPABILITY_
#define MOVEIT_MOVE_GROUP_GRASP_ACTION_CAPABILITY_

// plugin library (includes moveit related library)
#include <moveit/move_group/move_group_capability.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <shape_msgs/SolidPrimitive.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <actionlib/server/simple_action_server.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <moveit_grasps/grasp_generator.h>
#include <moveit_grasps/grasp_filter.h>
#include <moveit_grasps/grasp_data.h>
#include <moveit_grasps/grasp_planner.h>

#include <moveit_ros_grasp/GraspPlanAction.h>
#include "crop_octomap/CropObject.h"

namespace move_group
{
namespace{
  bool isStateValid(const planning_scene::PlanningScene* planning_scene, 
                    moveit_visual_tools::MoveItVisualToolsPtr visual_tools, 
                    robot_state::RobotState* robot_state, 
                    const robot_model::JointModelGroup* group, 
                    const double* ik_solution)
  {
    robot_state->setJointGroupPositions(group, ik_solution);
    robot_state->update();
    return !planning_scene->isStateColliding(*robot_state, group->getName());
  }
}

class MoveGroupGraspAction: public MoveGroupCapability
{
public:
  MoveGroupGraspAction();
  void initialize() override;

private:
  void loadVisual();
  void setupGraspPipeline();
  void executeCB(const moveit_ros_grasp::GraspPlanGoalConstPtr &goal);
  geometry_msgs::Pose normalizeQuaternion(geometry_msgs::Pose object_pose);
  bool generateGraspPlan(EigenSTL::vector_Isometry3d &grasp_waypoints, 
                         const geometry_msgs::Pose object_pose, 
                         const shape_msgs::SolidPrimitive object_shape);
  bool getIKSolution(const moveit::core::JointModelGroup* arm_jmg, 
                     const Eigen::Isometry3d& target_pose, 
                     robot_state::RobotState& solution, 
                     const std::string& link_name);
  bool planFullGrasp(std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates, 
                     moveit_grasps::GraspCandidatePtr& valid_grasp_candidate);
  void setACMFingerEntry(const std::string& object_name, bool allowed);

  std::unique_ptr<actionlib::SimpleActionServer<moveit_ros_grasp::GraspPlanAction>  > grasp_action_server_;
  moveit_ros_grasp::GraspPlanFeedback feedback_;
  moveit_ros_grasp::GraspPlanResult result_;

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  moveit_grasps::GraspGeneratorPtr grasp_generator_;
  moveit_grasps::GraspDataPtr grasp_data_;
  moveit_grasps::GraspPlannerPtr grasp_planner_;
  moveit_grasps::GraspFilterPtr grasp_filter_;

  const robot_model::JointModelGroup* arm_jmg_;
  robot_model::RobotModelConstPtr robot_model_;
  std::string ee_group_name_;
  std::string planning_group_name_;

  MoveGroupState grasp_state_;
  //void setGraspState(MoveGroupState state);
  ros::Publisher client_;

};
}

#endif

