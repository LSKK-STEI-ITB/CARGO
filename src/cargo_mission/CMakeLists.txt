cmake_minimum_required(VERSION 3.8)
project(cargo_mission)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cargo_core REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${cargo_core_INCLUDE_DIRS}
)

add_executable(cargo_mission_node
  src/main.cpp
  src/mission_node.cpp
)

ament_target_dependencies(cargo_mission_node
  rclcpp
  cargo_core
  mavros_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS cargo_mission_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
