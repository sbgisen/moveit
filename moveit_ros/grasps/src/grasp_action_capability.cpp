/*
 * Author: Joshua Supratman
 * Description: moveit plugin (MoveGroupCapability) for using moveit_grasps
 */

#include "grasp_action_capability.h"
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_grasp_capability/capability_names.h>


move_group::MoveGroupGraspAction::MoveGroupGraspAction()
  : MoveGroupCapability("GraspAction"), grasp_state_(IDLE)
{
}


void move_group::MoveGroupGraspAction::initialize()
{
    root_node_handle_.param("planning_group_name", planning_group_name_, std::string("arm"));
    root_node_handle_.param("ee_group_name", ee_group_name_, std::string("hand"));
    ROS_INFO_STREAM_NAMED(getName(), "End Effector: " << ee_group_name_);
    ROS_INFO_STREAM_NAMED(getName(), "Planning Group: " << planning_group_name_);

    loadVisual();
    setupGraspPipeline();

    grasp_action_server_.reset(new actionlib::SimpleActionServer<moveit_ros_grasp::GraspPlanAction>(
        root_node_handle_, GRASP_ACTION, boost::bind(&MoveGroupGraspAction::executeCB, this, _1), false));
    //grasp_action_server_->registerPreemptCallback(boost::bind(&MoveGroupGraspAction::preemptCB, this));
    grasp_action_server_->start();
}

void move_group::MoveGroupGraspAction::loadVisual()
{

    // Load the robot model
    robot_model_ = context_->planning_scene_monitor_->getRobotModel();
    arm_jmg_ = robot_model_->getJointModelGroup(planning_group_name_); //TODO: adapt jmg based on called planning group

    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(robot_model_->getModelFrame(),
                                                                   "/rviz_visual_tools",
                                                                   context_->planning_scene_monitor_));
    visual_tools_->loadMarkerPub();
    //visual_tools_->loadRobotStatePub("/display_robot_state");
    //visual_tools_->loadTrajectoryPub("/display_planned_path");
    //visual_tools_->loadSharedRobotState();
    //visual_tools_->enableBatchPublishing();
    visual_tools_->deleteAllMarkers();
    visual_tools_->removeAllCollisionObjects();
    visual_tools_->trigger();
}

void move_group::MoveGroupGraspAction::setupGraspPipeline()
{
    // ---------------------------------------------------------------------------------------------
    // Load grasp data specific to our robot
    grasp_data_.reset(new moveit_grasps::GraspData(root_node_handle_, ee_group_name_, visual_tools_->getRobotModel()));

    // ---------------------------------------------------------------------------------------------
    // Load grasp generator
    grasp_generator_.reset(new moveit_grasps::GraspGenerator(visual_tools_));

    // Set the ideal grasp orientation for scoring
    std::vector<double> ideal_grasp_rpy = { 0.0, 3.141, 0 }; // bin pick rpy
    grasp_generator_->setIdealTCPGraspPoseRPY(ideal_grasp_rpy);

    // Set custom grasp score weights
    moveit_grasps::GraspScoreWeights grasp_score_weights;
    grasp_score_weights.orientation_x_score_weight_ = 2.0;
    grasp_score_weights.orientation_y_score_weight_ = 2.0;
    grasp_score_weights.orientation_z_score_weight_ = 2.0;
    grasp_score_weights.translation_x_score_weight_ = 1.0;
    grasp_score_weights.translation_y_score_weight_ = 1.0;
    grasp_score_weights.translation_z_score_weight_ = 1.0;
    // Finger gripper specific weights.
    // Note that we do not need to set the suction gripper specific weights for our finger gripper.
    grasp_score_weights.depth_score_weight_ = 2.0;
    grasp_score_weights.width_score_weight_ = 2.0;
    grasp_generator_->setGraspScoreWeights(grasp_score_weights);

    // ---------------------------------------------------------------------------------------------
    // Load grasp filter
    grasp_filter_.reset(new moveit_grasps::GraspFilter(visual_tools_->getSharedRobotState(), visual_tools_));

    // ---------------------------------------------------------------------------------------------
    // Load grasp planner for approach, lift and retreat planning
    grasp_planner_.reset(new moveit_grasps::GraspPlanner(visual_tools_));
}



void move_group::MoveGroupGraspAction::executeCB(const moveit_ros_grasp::GraspPlanGoalConstPtr &goal)
{
    //setGraspState(PLANNING);

    // Delete previous markers
    visual_tools_->deleteAllMarkers();
    visual_tools_->removeAllCollisionObjects();
    visual_tools_->trigger();

    // Plan grasps
    geometry_msgs::Pose object_pose = goal->object_pose;
    shape_msgs::SolidPrimitive object_shape = goal->object_shape;
    EigenSTL::vector_Isometry3d waypoints;
    bool success = generateGraspPlan(waypoints, object_pose, object_shape);

    if(success){
      feedback_.pregrasp_pose = normalizeQuaternion(visual_tools_->convertPose(waypoints[0]));
      feedback_.approach_pose = normalizeQuaternion(visual_tools_->convertPose(waypoints[1]));
      feedback_.lift_pose     = normalizeQuaternion(visual_tools_->convertPose(waypoints[2]));
      feedback_.retreat_pose  = normalizeQuaternion(visual_tools_->convertPose(waypoints[3]));
      grasp_action_server_->publishFeedback(feedback_);
      ros::Duration(0.1).sleep();
    }

    result_.success = success;
    grasp_action_server_->setSucceeded(result_);
}

geometry_msgs::Pose move_group::MoveGroupGraspAction::normalizeQuaternion(geometry_msgs::Pose object_pose)
  {
    tf2::Quaternion quat;
    geometry_msgs::Quaternion orient = object_pose.orientation;

    tf2::fromMsg(orient, quat);
    quat = quat.normalized();
    orient = tf2::toMsg(quat);
    object_pose.orientation = orient;

    return object_pose;
}

bool move_group::MoveGroupGraspAction::generateGraspPlan(EigenSTL::vector_Isometry3d &grasp_waypoints, 
                                                         const geometry_msgs::Pose object_pose, 
                                                         const shape_msgs::SolidPrimitive object_shape)
{
    // -----------------------------------
    // Define object to grasp
    double object_x_depth  = object_shape.dimensions[0];
    double object_y_width  = object_shape.dimensions[1];
    double object_z_height = object_shape.dimensions[2];
    std::string object_name = "object_link";

    //visualize object
    visual_tools_->publishCollisionCuboid(object_pose, object_x_depth, object_y_width, object_z_height, object_name, rviz_visual_tools::RED);
    visual_tools_->publishAxis(object_pose, rviz_visual_tools::MEDIUM);
    visual_tools_->trigger();
   

    setACMFingerEntry(object_name, true);

    // -----------------------------------
    // Generate grasp candidates
    std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates;

    // Configure the desired types of grasps
    moveit_grasps::GraspCandidateConfig grasp_generator_config = moveit_grasps::GraspCandidateConfig();
    grasp_generator_config.disableAll();
    grasp_generator_config.enable_face_grasps_ = true;
    grasp_generator_config.generate_y_axis_grasps_ = true;
    grasp_generator_config.generate_x_axis_grasps_ = true;
    grasp_generator_config.generate_z_axis_grasps_ = true;

    if (!grasp_generator_->generateGrasps(visual_tools_->convertPose(object_pose), object_x_depth, object_y_width,
                                          object_z_height, grasp_data_, grasp_candidates, grasp_generator_config))
    {
      ROS_ERROR_NAMED(getName(), "Grasp generator failed to generate any valid grasps");
      return false;
    }

    // --------------------------------------------
    // Generating a seed state for filtering grasps
    robot_state::RobotStatePtr seed_state(new robot_state::RobotState(*visual_tools_->getSharedRobotState()));
    Eigen::Isometry3d eef_mount_grasp_pose =
        visual_tools_->convertPose(object_pose) * grasp_data_->tcp_to_eef_mount_.inverse();
    if (!getIKSolution(arm_jmg_, eef_mount_grasp_pose, *seed_state, grasp_data_->parent_link_->getName()))
    {
      ROS_WARN_NAMED(getName(), "The ideal seed state is not reachable. Using start state as seed.");
    }

    // --------------------------------------------
    // Filtering grasps
    // Note: This step also solves for the grasp and pre-grasp states and stores them in grasp candidates)
    bool filter_pregrasps = true;
    if (!grasp_filter_->filterGrasps(grasp_candidates, context_->planning_scene_monitor_, arm_jmg_, seed_state, filter_pregrasps))
    {
      ROS_ERROR_NAMED(getName(), "Filter grasps failed");
      return false;
    }
    if (!grasp_filter_->removeInvalidAndFilter(grasp_candidates))
    {
      ROS_WARN_NAMED(getName(), "Grasp filtering removed all grasps");
      return false;
    }
    ROS_INFO_STREAM_NAMED(getName(), "" << grasp_candidates.size() << " remain after filtering");

    // Plan free-space approach, cartesian approach, lift and retreat trajectories
    moveit_grasps::GraspCandidatePtr selected_grasp_candidate;
    if (!planFullGrasp(grasp_candidates, selected_grasp_candidate))
    {
      ROS_ERROR_NAMED(getName(), "Failed to plan grasp motions");
      return false;
    }

    // --------------------------------------------
    // Returning grasp waypoints
    EigenSTL::vector_Isometry3d waypoints;
    moveit_grasps::GraspGenerator::getGraspWaypoints(selected_grasp_candidate, waypoints);
    grasp_waypoints = waypoints;

    // Visualize waypoints
    visual_tools_->publishAxisLabeled(waypoints[0], "pregrasp");
    visual_tools_->publishAxisLabeled(waypoints[1], "grasp");
    visual_tools_->publishAxisLabeled(waypoints[2], "lifted");
    visual_tools_->publishAxisLabeled(waypoints[3], "retreat");
    visual_tools_->trigger();

    //setACMFingerEntry(object_name, false);

    return true;
}

bool move_group::MoveGroupGraspAction::getIKSolution(const moveit::core::JointModelGroup* arm_jmg, 
                                                     const Eigen::Isometry3d& target_pose,
                                                     robot_state::RobotState& solution, 
                                                     const std::string& link_name)
{
    boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRW> ls(
        new planning_scene_monitor::LockedPlanningSceneRW(context_->planning_scene_monitor_));

    moveit::core::GroupStateValidityCallbackFn constraint_fn = boost::bind(
        &isStateValid, static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get(), visual_tools_, _1, _2, _3);

    // seed IK call with current state
    solution = (*ls)->getCurrentState();

    // Solve IK problem for arm
    // disable explicit restarts to guarantee close solution if one exists
    const double timeout = 0.1;
    return solution.setFromIK(arm_jmg, target_pose, link_name, timeout, constraint_fn);
}


bool move_group::MoveGroupGraspAction::planFullGrasp(std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates,
                                                     moveit_grasps::GraspCandidatePtr& valid_grasp_candidate)
{
    moveit::core::RobotStatePtr current_state;
    {
      boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRW> ls(
          new planning_scene_monitor::LockedPlanningSceneRW(context_->planning_scene_monitor_));
      current_state.reset(new moveit::core::RobotState((*ls)->getCurrentState()));
    }

    bool success = false;
    for (; !grasp_candidates.empty(); grasp_candidates.erase(grasp_candidates.begin()))
    {
      valid_grasp_candidate = grasp_candidates.front();
      valid_grasp_candidate->getPreGraspState(current_state);
      if (!grasp_planner_->planApproachLiftRetreat(valid_grasp_candidate, current_state, context_->planning_scene_monitor_, false))
      {
        ROS_INFO_NAMED(getName(), "failed to plan approach lift retreat");
        continue;
      }
        
      success = true;
      break;
    }
    return success;
}

void move_group::MoveGroupGraspAction::setACMFingerEntry(const std::string& object_name, bool allowed)
{
    planning_scene_monitor::LockedPlanningSceneRW scene(context_->planning_scene_monitor_);  // Lock planning scene

    // Get links of end effector
    const std::vector<std::string>& ee_links = grasp_data_->ee_jmg_->getLinkModelNames();

    // Set collision checking between fingers and object
    for (std::size_t i = 0; i < ee_links.size(); ++i)
    {
      scene->getAllowedCollisionMatrixNonConst().setEntry(object_name, ee_links[i], allowed);
    }
}

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupGraspAction, move_group::MoveGroupCapability)
