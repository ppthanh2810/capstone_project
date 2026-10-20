cmake_minimum_required(VERSION 3.8)
project(stage2_slam)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(scan_rotate src/scan_rotate.cpp)
target_compile_features(scan_rotate PUBLIC cxx_std_17)
ament_target_dependencies(scan_rotate rclcpp sensor_msgs)

install(TARGETS scan_rotate
  DESTINATION lib/${PROJECT_NAME}
)

# Lõi localization nâng cấp (CSM + health), không phụ thuộc ROS.
add_library(stage2_localization_core STATIC
  src/scan_match_initializer.cpp
  src/localization_health.cpp
  src/scan_map_consistency.cpp
)
target_include_directories(stage2_localization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(stage2_localization_core PUBLIC cxx_std_17)
set_target_properties(stage2_localization_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Node ROS 2 (scan_match_initializer_node.cpp còn chứa hàm chuyển msg dùng chung với monitor).
set(LOCALIZATION_ROS_DEPS rclcpp sensor_msgs geometry_msgs nav_msgs diagnostic_msgs lifecycle_msgs
  tf2 tf2_ros)
add_library(stage2_localization_nodes STATIC
  src/scan_match_initializer_node.cpp
)
target_link_libraries(stage2_localization_nodes stage2_localization_core)
ament_target_dependencies(stage2_localization_nodes ${LOCALIZATION_ROS_DEPS})
set_target_properties(stage2_localization_nodes PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(scan_match_initializer src/scan_match_initializer_main.cpp)
target_link_libraries(scan_match_initializer stage2_localization_nodes)
ament_target_dependencies(scan_match_initializer ${LOCALIZATION_ROS_DEPS})

add_executable(localization_monitor src/localization_monitor_node.cpp)
target_link_libraries(localization_monitor stage2_localization_nodes)
ament_target_dependencies(localization_monitor ${LOCALIZATION_ROS_DEPS})

install(TARGETS scan_match_initializer localization_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
