cmake_minimum_required(VERSION 3.5)
project(quad_swerve_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

add_library(quad_swerve_controller SHARED
  src/quad_swerve_controller.cpp
  src/motor_interface.cpp
)

ament_target_dependencies(quad_swerve_controller
  rclcpp
  rclcpp_lifecycle
  controller_interface
  controller_manager
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

install(TARGETS quad_swerve_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
