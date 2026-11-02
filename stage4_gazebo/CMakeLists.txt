cmake_minimum_required(VERSION 3.8)
project(stage4_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Mô hình driver ZLAC8015D, không phụ thuộc ROS (dùng lại được trong unit test / plugin Gazebo sau này)
add_library(zlac_driver_model src/s0_control/zlac_driver_model.cpp)
target_include_directories(zlac_driver_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(zlac_driver_model PUBLIC cxx_std_17)

# s0: zlac_sim (thay zlac8015d_move)
add_executable(zlac_sim src/s0_control/zlac_sim_node.cpp)
target_link_libraries(zlac_sim zlac_driver_model)
ament_target_dependencies(zlac_sim rclcpp std_msgs geometry_msgs sensor_msgs)
target_compile_features(zlac_sim PUBLIC cxx_std_17)

install(TARGETS zlac_driver_model
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS zlac_sim DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS test/s0_check.py DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    launch
    config
    urdf
    worlds
    rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_zlac_driver_model test/test_zlac_driver_model.cpp)
  target_link_libraries(test_zlac_driver_model zlac_driver_model)
endif()

ament_package()
