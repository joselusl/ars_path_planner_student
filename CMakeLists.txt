cmake_minimum_required(VERSION 3.0.0)
project(ars_path_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  message_generation
)

add_service_files(
  FILES
    GetCollisionFreePath.srv
    CheckPathCollisionFree.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS rospy std_msgs geometry_msgs visualization_msgs nav_msgs message_generation ars_lib_helpers
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
