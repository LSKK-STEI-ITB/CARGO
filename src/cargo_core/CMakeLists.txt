cmake_minimum_required(VERSION 3.8)
project(cargo_core)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------------------------
# Dependencies
# -------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


# -------------------------------------------------------------------------
# Include directories
# -------------------------------------------------------------------------
include_directories(
  include
)

# -------------------------------------------------------------------------
# Libraries inside cargo_core
# -------------------------------------------------------------------------
add_library(cargo_lib SHARED src/cargo_node.cpp)
add_library(cargo_control SHARED src/cargo_control.cpp)

ament_target_dependencies(cargo_lib
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  mavros_msgs
)

ament_target_dependencies(cargo_control
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  mavros_msgs
)

# -------------------------------------------------------------------------
# Install internal libs
# -------------------------------------------------------------------------
install(TARGETS cargo_lib cargo_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# -------------------------------------------------------------------------
# Executable for your pilot node
# -------------------------------------------------------------------------
add_executable(cargo_node src/main.cpp)

ament_target_dependencies(cargo_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  mavros_msgs
)

target_link_libraries(cargo_node cargo_lib cargo_control)

install(TARGETS cargo_node
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------------------------------------------------
# 🔥 INSTALL HEADERS FOR PLUGIN PACKAGES
# -------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(cargo_lib cargo_control)

# -------------------------------------------------------------------------
# Tests
# -------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

