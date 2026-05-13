cmake_minimum_required(VERSION 3.16)
project(luna_control LANGUAGES CXX)

#ccache stuff
find_program(CCACHE ccache)
if(CCACHE)
  set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE})
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE})
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # disables -std=gnu++17, enforces -std=c++17

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow -Werror=format)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS_PUBLIC
    controller_interface
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
)

set(THIS_PACKAGE_INCLUDE_DEPENDS_PRIVATE
    rcpputils
    realtime_tools
    tf2
    tf2_msgs
    geometry_msgs
    nav_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS 
  ${THIS_PACKAGE_INCLUDE_DEPENDS_PUBLIC}
  ${THIS_PACKAGE_INCLUDE_DEPENDS_PRIVATE})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(luna_controller_parameters
  src/luna_controller_parameter.yaml
)

#set up libraries
add_library(luna_control SHARED
  src/LunaController.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)

add_library(luna_can_lib SHARED
  src/LunaCan.cpp
  src/SparkMax.cpp
  src/SparkBase.cpp
)

target_compile_definitions(luna_control PRIVATE
  RCPPUTILS_VERSION_MAJOR=${rcpputils_VERSION_MAJOR}
  RCPPUTILS_VERSION_MINOR=${rcpputils_VERSION_MINOR}
)

#executables
add_executable(luna_debug src/luna_debug.cpp)
add_executable(swerve_debug src/swerve_debug.cpp)

target_link_libraries(luna_control PUBLIC luna_can_lib)

target_link_libraries(luna_debug
  PRIVATE luna_control
)

target_link_libraries(swerve_debug
  PRIVATE luna_control
)

target_include_directories(luna_control PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/luna_control>
)
target_include_directories(luna_can_lib PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/luna_control>
)

target_link_libraries(luna_control PUBLIC luna_controller_parameters)
ament_target_dependencies(luna_control PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS_PUBLIC})
target_link_libraries(luna_control PRIVATE
  ${tf2_LIBRARIES}
  ${tf2_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${realtime_tools_LIBRARIES}
  ${rcpputils_LIBRARIES}
)
target_include_directories(luna_control PRIVATE
  ${tf2_INCLUDE_DIRS}
  ${tf2_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${realtime_tools_INCLUDE_DIRS}
  ${rcpputils_INCLUDE_DIRS}
)
ament_target_dependencies(luna_can_lib PUBLIC rclcpp hardware_interface)
target_link_libraries(luna_control PRIVATE
  ${rcpputils_LIBRARIES}
)
target_include_directories(luna_control PRIVATE
  ${rcpputils_INCLUDE_DIRS}
)
target_compile_definitions(luna_control PRIVATE "LUNA_CONTROL_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface luna_plugin.xml)
pluginlib_export_plugin_description_file(hardware_interface luna_can.xml)

install(
  DIRECTORY include/
  DESTINATION include/luna_control
)
install(TARGETS luna_control luna_can_lib luna_controller_parameters luna_debug swerve_debug
  EXPORT export_luna_control
  RUNTIME DESTINATION lib/luna_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(
  FILES luna_can.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_luna_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS_PUBLIC})
ament_package()