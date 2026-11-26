cmake_minimum_required(VERSION 3.8)
project(stage5_benchmark)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(stage3_navigation REQUIRED)     # dwa_core, path_blockage, replan_supervisor (ROS-free)
find_package(stage4_gazebo REQUIRED)         # zlac_driver_model (ROS-free)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# Fast 2D simulator: CHÍNH code điều hướng của robot (DwaCore, ReplanSupervisor, NavFn, Costmap2D, ZlacDriverModel)
add_library(fast2d SHARED src/fast2d.cpp)
target_compile_features(fast2d PUBLIC cxx_std_17)
target_include_directories(fast2d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(fast2d
  stage3_navigation::dwa_core stage3_navigation::replan_supervisor stage3_navigation::path_blockage
  stage4_gazebo::zlac_driver_model)
ament_target_dependencies(fast2d nav2_costmap_2d nav2_navfn_planner geometry_msgs)
install(TARGETS fast2d LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

# Python module: stage5_benchmark.fast2d
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})
pybind11_add_module(fast2d_py MODULE src/fast2d_py.cpp)
set_target_properties(fast2d_py PROPERTIES OUTPUT_NAME fast2d)
target_link_libraries(fast2d_py PRIVATE fast2d)
install(TARGETS fast2d_py DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")

install(PROGRAMS
  scripts/gen_scenarios.py scripts/pilot.py scripts/gazebo_episode.py
  scripts/a_stage_tests.py scripts/tune_baseline.py scripts/sim2gz_compare.py scripts/eval_split.py
  scripts/replan_study.py scripts/replan_compare.py scripts/replan_report.py scripts/replan_figures.py
  scripts/sensitivity_study.py scripts/gen_stress.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  # R2 Local Avoidance First (ReplanSupervisor) + chẩn đoán không gian trống + dừng an toàn (DwaCore), 2026-11-27
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_replan_supervisor test/test_replan_supervisor.cpp)
  target_link_libraries(test_replan_supervisor
    stage3_navigation::dwa_core stage3_navigation::replan_supervisor stage3_navigation::path_blockage)
  ament_target_dependencies(test_replan_supervisor nav2_costmap_2d)
endif()

ament_package()
