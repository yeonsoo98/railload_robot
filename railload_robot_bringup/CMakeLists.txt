cmake_minimum_required(VERSION 3.5)
project(railload_robot_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(joy_linux REQUIRED)
find_package(teleop_twist_joy REQUIRED)
find_package(realsense2_camera REQUIRED)
find_package(realsense2_description REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)

include_directories(include)

#add_library(SHARED
#  src/joy_commander/joy_commander.cpp
#)

################################################################################
# Build
################################################################################
add_executable(joy_commander
  src/joy_commander/main.cpp
  src/joy_commander/joy_commander.cpp
)
ament_target_dependencies(joy_commander
  rclcpp sensor_msgs
)

################################################################################
# Install
################################################################################
install(
  DIRECTORY config include launch src
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS joy_commander
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Macro for ament package
################################################################################
ament_package()