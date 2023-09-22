cmake_minimum_required(VERSION 3.0.2)
project(tdf_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  moveit_commander
  moveit_core
  moveit_fake_controller_manager
  moveit_msgs
  moveit_ros_move_group
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_robot_interaction
  moveit_msgs
  cv_bridge
  image_transport
  move_base_msgs
  actionlib
  tf2
  tf2_ros
  tf2_geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/tdf_simulation_node.cpp)
add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node ${catkin_LIBRARIES})

add_executable(control_node src/control_node.cpp)
target_link_libraries(control_node ${catkin_LIBRARIES})
