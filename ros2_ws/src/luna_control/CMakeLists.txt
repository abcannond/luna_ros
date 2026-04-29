cmake_minimum_required(VERSION 3.16)
project(luna_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow -Werror=format)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
add_compile_definitions(RCPPUTILS_VERSION_MAJOR=${rcpputils_VERSION_MAJOR})
add_compile_definitions(RCPPUTILS_VERSION_MINOR=${rcpputils_VERSION_MINOR})

generate_parameter_library(luna_controller_parameters
  src/luna_controller_parameter.yaml
)

#important to include SparkMax stuff so that LunaCAN can find it 
add_library(luna_control SHARED
  src/LunaController.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
  src/SparkMax.cpp
  src/SparkBase.cpp
)

#executables
add_executable(luna_can src/luna_can.cpp)
add_executable(luna_linaks src/luna_linaks.cpp)
add_executable(luna_debug src/luna_debug.cpp)

target_link_libraries(luna_can
  luna_control
)

target_link_libraries(luna_debug
  luna_control
)

target_link_libraries(luna_linaks
  luna_control
)

ament_target_dependencies(luna_can
  rclcpp
)

ament_target_dependencies(luna_debug
  rclcpp
)

ament_target_dependencies(luna_linaks
  rclcpp
)

target_compile_features(luna_control PUBLIC cxx_std_17)
target_include_directories(luna_control PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/luna_control>
)
target_link_libraries(luna_control PUBLIC luna_controller_parameters)
ament_target_dependencies(luna_control PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(luna_control PRIVATE "LUNA_CONTROL_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface luna_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/luna_control
)
install(TARGETS luna_control luna_controller_parameters luna_can luna_debug
  EXPORT export_luna_control
  RUNTIME DESTINATION lib/luna_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_luna_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()