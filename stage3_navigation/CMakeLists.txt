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

# Plugin nav2_core::Controller tự viết (Dynamic Window): libdwb_controller.so, nạp bởi controller_server
# qua pluginlib (dwb_controller_plugin.xml). Chọn bằng planning.launch.py local_planner:=custom.
add_library(dwb_controller SHARED src/dwb_controller.cpp)
target_compile_features(dwb_controller PUBLIC cxx_std_17)
target_include_directories(dwb_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(dwb_controller
  rclcpp rclcpp_lifecycle geometry_msgs nav_msgs visualization_msgs nav2_core nav2_costmap_2d
  nav2_util pluginlib tf2 tf2_ros tf2_geometry_msgs)
pluginlib_export_plugin_description_file(nav2_core dwb_controller_plugin.xml)

install(TARGETS dwb_controller
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
