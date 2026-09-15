cmake_minimum_required(VERSION 3.8)
project(zlac8015d_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED IMPORTED_TARGET libmodbus)

# Library
add_library(zlac8015d SHARED
  src/zlac8015d.cpp
)

target_include_directories(zlac8015d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(zlac8015d
  PkgConfig::LIBMODBUS
)

# Install library
install(
  TARGETS zlac8015d
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Export to other ROS2 packages
ament_export_include_directories(include)
ament_export_libraries(zlac8015d)

ament_package()