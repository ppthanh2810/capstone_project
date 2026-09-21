cmake_minimum_required(VERSION 3.8)
project(stage0_ps4_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(zlac8015d_hw REQUIRED)

# Nodes
add_executable(m_to_m src/m_to_m.cpp)
ament_target_dependencies(m_to_m rclcpp std_msgs geometry_msgs)
target_compile_features(m_to_m PUBLIC cxx_std_17)

add_executable(move src/move.cpp)
ament_target_dependencies(move rclcpp std_msgs zlac8015d_hw geometry_msgs)
target_compile_features(move PUBLIC cxx_std_17)

# Install nodes
install(
  TARGETS
  m_to_m
  move
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