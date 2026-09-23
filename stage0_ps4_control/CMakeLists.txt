cmake_minimum_required(VERSION 3.8)
project(stage0_ps4_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(zlac8015d_hw REQUIRED)

# m_to_m
add_executable(m_to_m src/m_to_m.cpp)
ament_target_dependencies(
  m_to_m
  rclcpp
  std_msgs
  geometry_msgs
)
target_compile_features(m_to_m PUBLIC cxx_std_17)

# move
add_executable(move src/move.cpp)
ament_target_dependencies(
  move
  rclcpp
  std_msgs
  geometry_msgs
  zlac8015d_hw
)
target_compile_features(move PUBLIC cxx_std_17)

# zlac8015d_move
add_executable(zlac8015d_move src/zlac8015d_move.cpp)
ament_target_dependencies(
  zlac8015d_move
  rclcpp
  geometry_msgs
  nav_msgs
  zlac8015d_hw
)

# odom
add_executable(odom src/odom.cpp)
ament_target_dependencies(
  odom
  rclcpp
  geometry_msgs
  nav_msgs
  std_srvs
  tf2_ros
)
target_compile_features(odom PUBLIC cxx_std_17)

# Install nodes
install(
  TARGETS
    m_to_m
    move
    zlac8015d_move
    odom
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config
install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()