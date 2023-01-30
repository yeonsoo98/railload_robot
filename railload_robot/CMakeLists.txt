cmake_minimum_required(VERSION 3.5)
project(railload_robot)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
    # "action/ActionName.action"
    # "srv/SrvName.srv"
# )

# install(
#   DIRECTORY action srv
#   DESTINATION share/${PROJECT_NAME}
# )

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()