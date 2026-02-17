cmake_minimum_required(VERSION 3.8)
project(depth_to_pointcloud)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})


# Node executable
add_executable(depth_to_pointcloud_node src/depth_to_pointcloud_node.cpp)
ament_target_dependencies(depth_to_pointcloud_node
  rclcpp
  sensor_msgs
  Eigen3
)

# Install executable
install(TARGETS depth_to_pointcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

# Enable testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export package
ament_package()
