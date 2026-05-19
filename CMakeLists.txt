cmake_minimum_required(VERSION 3.8)
project(robot_ik_layer)

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
find_package(pinocchio REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(robot_safety_layer REQUIRED)

include_directories(
  include
)

add_executable(ik_node src/franka_ijk.cpp)

ament_target_dependencies(ik_node
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
  pinocchio
  robot_safety_layer
)


install(TARGETS ik_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# install(DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/config)

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
