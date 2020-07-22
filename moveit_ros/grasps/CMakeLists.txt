cmake_minimum_required(VERSION 3.0.2)
project(moveit_ros_grasp)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED thread system)
find_package(OpenMP)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    shape_msgs
    actionlib_msgs
    message_generation
    actionlib
    roscpp
    moveit_grasps
    tf
    tf_conversions
    tf2
    tf2_ros
    tf2_geometry_msgs
    trajectory_msgs
    moveit_core
    moveit_msgs
    moveit_ros_move_group
    moveit_ros_planning
    moveit_ros_planning_interface
)

add_action_files(
    DIRECTORY
    action

    FILES
    GraspPlan.action
)

generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
   geometry_msgs
   shape_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    actionlib_msgs
    actionlib
    roscpp
  DEPENDS
    EIGEN3
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
link_directories(${catkin_LIBRARY_DIRS})

set(MOVEIT_LIB_NAME moveit_move_group_grasp_capability)
add_library(${MOVEIT_LIB_NAME}
    src/grasp_action_capability.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} moveit_grasps moveit_grasps_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(FILES grasp_capability_plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
