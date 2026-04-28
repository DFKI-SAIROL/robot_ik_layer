cmake_minimum_required(VERSION 3.8)
project(franka_safety_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(franka_custom_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pinocchio REQUIRED)
find_package(visualization_msgs REQUIRED)


include_directories(
  include
)


add_executable(safety_node src/franka_ijk.cpp src/safety_layer.cpp src/vis.cpp)

ament_target_dependencies(safety_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  trajectory_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  franka_custom_msgs
  #moveit_core
  #moveit_ros_planning
  #moveit_ros_planning_interface
  #moveit_ros_move_group
  pinocchio
)

install(TARGETS safety_node
  DESTINATION lib/${PROJECT_NAME}
)



install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
