cmake_minimum_required(VERSION 3.8)
project(stage1_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(zlac8015d_hw REQUIRED)
find_package(sensor_msgs REQUIRED)


# zlac8015d_odom
add_executable(zlac8015d_odom src/zlac8015d_odom.cpp)
ament_target_dependencies(
  zlac8015d_odom
  rclcpp
  geometry_msgs
  nav_msgs
  std_srvs
  tf2_ros
  zlac8015d_hw
)
target_compile_features(zlac8015d_odom PUBLIC cxx_std_17)

# imu_odom
add_executable(imu_odom src/imu_odom.cpp)
ament_target_dependencies(
  imu_odom
  rclcpp
  sensor_msgs
  nav_msgs
)
target_compile_features(imu_odom PUBLIC cxx_std_17)

# Install nodes
install(
  TARGETS
    zlac8015d_odom
    imu_odom
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