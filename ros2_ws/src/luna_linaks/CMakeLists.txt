cmake_minimum_required(VERSION 3.16)
project(luna_linaks LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

add_library(luna_linaks SHARED
  src/luna_linaks.cpp
)
target_compile_features(luna_linaks PUBLIC cxx_std_17)
target_include_directories(luna_linaks PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/luna_linaks>
)

install(DIRECTORY include/ DESTINATION include/luna_linaks)
install(TARGETS luna_linaks
  EXPORT export_luna_linaks
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_luna_linaks HAS_LIBRARY_TARGET)
ament_package()
