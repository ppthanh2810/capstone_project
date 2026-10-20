cmake_minimum_required(VERSION 3.8)
project(stage3_navigation)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
# dwb_controller (plugin bộ điều khiển cục bộ tự viết)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)
# fusion_layer (plugin costmap camera + LiDAR, M2)
find_package(sensor_msgs REQUIRED)

# Bridge RViz Goal Pose -> ComputePathToPose -> /global_path; tùy chọn -> FollowPath (DWB) khi được kích hoạt.
add_executable(goal_path_bridge src/goal_path_bridge.cpp)
target_compile_features(goal_path_bridge PUBLIC cxx_std_17)
ament_target_dependencies(goal_path_bridge
  rclcpp rclcpp_action geometry_msgs nav_msgs nav2_msgs tf2 tf2_ros tf2_geometry_msgs
  std_msgs std_srvs diagnostic_msgs)

install(TARGETS goal_path_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Lõi thuật toán Dynamic Window (không phụ thuộc ROS node, chỉ nav2_costmap_2d::Costmap2D): dùng bởi
# plugin dwb_controller.
add_library(dwa_core SHARED src/dwa_core.cpp)
target_compile_features(dwa_core PUBLIC cxx_std_17)
target_include_directories(dwa_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(dwa_core nav2_costmap_2d)

# Plugin nav2_core::Controller tự viết (Dynamic Window): libdwb_controller.so, nạp bởi controller_server
# qua pluginlib (dwb_controller_plugin.xml). Chọn bằng navigation.launch.py local_planner:=custom.
add_library(dwb_controller SHARED src/dwb_controller.cpp)
target_compile_features(dwb_controller PUBLIC cxx_std_17)
target_include_directories(dwb_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(dwb_controller dwa_core)
ament_target_dependencies(dwb_controller
  rclcpp rclcpp_lifecycle geometry_msgs nav_msgs visualization_msgs diagnostic_msgs nav2_core
  nav2_costmap_2d nav2_util pluginlib tf2 tf2_ros tf2_geometry_msgs)
pluginlib_export_plugin_description_file(nav2_core dwb_controller_plugin.xml)

# Lõi hợp nhất camera + LiDAR (M2, không phụ thuộc ROS) + plugin costmap nav2_costmap_2d::Layer:
# libfusion_layer.so, nạp bởi local_costmap qua pluginlib (fusion_layer_plugin.xml). Bật bằng
# navigation.launch.py camera_layer:=fusion (hoặc camera:=true).
add_library(camera_fusion SHARED src/camera_fusion.cpp)
target_compile_features(camera_fusion PUBLIC cxx_std_17)
target_include_directories(camera_fusion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(fusion_layer SHARED src/fusion_layer.cpp)
target_compile_features(fusion_layer PUBLIC cxx_std_17)
target_link_libraries(fusion_layer camera_fusion)
ament_target_dependencies(fusion_layer
  rclcpp rclcpp_lifecycle nav2_costmap_2d pluginlib sensor_msgs diagnostic_msgs geometry_msgs tf2 tf2_ros
  tf2_geometry_msgs)
pluginlib_export_plugin_description_file(nav2_costmap_2d fusion_layer_plugin.xml)

install(TARGETS dwa_core dwb_controller camera_fusion fusion_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
