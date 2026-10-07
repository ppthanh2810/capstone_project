cmake_minimum_required(VERSION 3.8)
project(imu_bno055)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Node
add_executable(bno055_node src/bno055_node.cpp)
ament_target_dependencies(bno055_node rclcpp sensor_msgs)

target_include_directories(bno055_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(bno055_node PUBLIC cxx_std_17)

# Install node
install(
  TARGETS
  bno055_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch
install(
  DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()