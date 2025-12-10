cmake_minimum_required(VERSION 3.8)
project(cargo_safety)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(include) 

# ----------------------------
# Build library
# ----------------------------
add_library(safety_checker SHARED src/safety_checker.cpp)

ament_target_dependencies(safety_checker
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  mavros_msgs
)

# ----------------------------
# Build executable
# ----------------------------
add_executable(safety_node src/main.cpp)

ament_target_dependencies(safety_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  mavros_msgs
)

target_link_libraries(safety_node safety_checker)

# ----------------------------
# Install BOTH: library + executable
# ----------------------------
install(
  TARGETS safety_checker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS safety_node
  DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------
# Tests
# ----------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

