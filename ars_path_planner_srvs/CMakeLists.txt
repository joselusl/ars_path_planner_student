cmake_minimum_required(VERSION 3.5)
project(ars_path_planner_srvs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetCollisionFreePath.srv"
  "srv/CheckPathCollisionFree.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

# Add these lines after the rosidl_generate_interfaces call
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()