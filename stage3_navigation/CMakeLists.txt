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

# Bridge RViz Goal Pose -> ComputePathToPose -> /global_path; tùy chọn -> FollowPath (DWB) khi được kích hoạt.
add_executable(goal_path_bridge src/goal_path_bridge.cpp)
target_compile_features(goal_path_bridge PUBLIC cxx_std_17)
ament_target_dependencies(goal_path_bridge
  rclcpp rclcpp_action geometry_msgs nav_msgs nav2_msgs tf2 tf2_ros tf2_geometry_msgs
  std_msgs std_srvs diagnostic_msgs)

install(TARGETS goal_path_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Lõi thuật toán Dynamic Window (không phụ thuộc ROS node, chỉ nav2_costmap_2d::Costmap2D): dùng chung cho
# plugin, unit test và mô phỏng kín vòng.
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

# Mô phỏng kín vòng KHÔNG cần ROS graph (động học có trễ + giới hạn gia tốc, costmap tổng hợp): benchmark lõi.
add_executable(dwa_closed_loop_sim test/dwa_closed_loop_sim.cpp test/sim_world.cpp)
target_include_directories(dwa_closed_loop_sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
target_link_libraries(dwa_closed_loop_sim dwa_core)
ament_target_dependencies(dwa_closed_loop_sim nav2_costmap_2d)
install(TARGETS dwa_closed_loop_sim DESTINATION lib/${PROJECT_NAME})

install(TARGETS dwa_core dwb_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config rviz test/offline
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

  # Unit test lõi DWA (không cần ROS graph) + kịch bản kín vòng tất định.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dwa_core test/test_dwa_core.cpp test/sim_world.cpp)
  target_include_directories(test_dwa_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_link_libraries(test_dwa_core dwa_core)
  ament_target_dependencies(test_dwa_core nav2_costmap_2d)

  # Test plugin qua lớp nav2_core::Controller thật (lifecycle node + Costmap2DROS + TF buffer, không có
  # controller_server, không publish cmd_vel): lifecycle, setPlan, TF/odom cũ, tham số, speed limit.
  # ENV: graph ROS cách ly (domain riêng + chỉ localhost) — test tạo node/topic thật trong process.
  ament_add_gtest(test_dwb_controller_plugin test/test_dwb_controller_plugin.cpp TIMEOUT 120
    ENV ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1)
  target_link_libraries(test_dwb_controller_plugin dwb_controller dwa_core)
  ament_target_dependencies(test_dwb_controller_plugin
    rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util tf2_ros tf2_geometry_msgs nav_msgs
    geometry_msgs pluginlib)
endif()

ament_package()
