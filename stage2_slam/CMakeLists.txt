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

# Lõi localization nâng cấp (CSM + health), không phụ thuộc ROS -> unit test được.
add_library(stage2_localization_core STATIC
  src/scan_match_initializer.cpp
  src/localization_health.cpp
  src/synthetic_world.cpp
)
target_include_directories(stage2_localization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(stage2_localization_core PUBLIC cxx_std_17)
set_target_properties(stage2_localization_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Node ROS 2 (thư viện riêng để node test chạy in-process).
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

# Benchmark offline trên dữ liệu tổng hợp (không ROS, không phần cứng).
add_executable(csm_benchmark src/csm_benchmark.cpp)
target_link_libraries(csm_benchmark stage2_localization_core)

install(TARGETS csm_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scan_match_initializer test/test_scan_match_initializer.cpp TIMEOUT 300)
  target_link_libraries(test_scan_match_initializer stage2_localization_core)
  ament_add_gtest(test_localization_health test/test_localization_health.cpp)
  target_link_libraries(test_localization_health stage2_localization_core)
  # Node test: tự cô lập (ROS_DOMAIN_ID riêng, localhost, namespace /csm_test) trong main().
  ament_add_gtest(test_scan_match_initializer_node test/test_scan_match_initializer_node.cpp
    SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 300)
  target_link_libraries(test_scan_match_initializer_node stage2_localization_nodes gtest)
  ament_target_dependencies(test_scan_match_initializer_node ${LOCALIZATION_ROS_DEPS})
  # Launch: tĩnh (không chạy phần cứng) + kiểm tra SetRemap bằng node giả trong domain cô lập.
  # Đăng ký trực tiếp bằng ament_add_test thay vì ament_add_pytest_test: bước kiểm tra phiên bản
  # pytest lúc configure crash khi có plugin pytest bên ngoài không tương thích (vd. anyio trong
  # ~/.local) và test bị bỏ qua âm thầm. PYTEST_DISABLE_PLUGIN_AUTOLOAD tránh nạp các plugin đó.
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(_launch_test_xml "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/test_localization_launch.xunit.xml")
  ament_add_test(test_localization_launch
    COMMAND ${Python3_EXECUTABLE} -m pytest
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_localization_launch.py
      -p no:cacheprovider --junit-xml=${_launch_test_xml} --junit-prefix=${PROJECT_NAME}
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
    RESULT_FILE ${_launch_test_xml}
    TIMEOUT 120)
endif()

ament_package()
