cmake_minimum_required(VERSION 3.10)
project(lio_scan_monitor)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/ros2_compat.cmake)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(scan_analysis
  src/scan_analysis.cpp
)
target_compile_features(scan_analysis PUBLIC cxx_std_17)
target_include_directories(scan_analysis PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(terminal_display
  src/terminal_display.cpp
)
target_compile_features(terminal_display PUBLIC cxx_std_17)
target_include_directories(terminal_display PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(terminal_display scan_analysis)

add_executable(lio_scan_monitor_node
  src/lio_scan_monitor_node.cpp
)
target_compile_features(lio_scan_monitor_node PUBLIC cxx_std_17)
target_link_libraries(lio_scan_monitor_node
  PRIVATE
    scan_analysis
    terminal_display
    rclcpp::rclcpp
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_ros::tf2_ros
)
target_link_ros2_package_compat(lio_scan_monitor_node builtin_interfaces)
target_link_ros2_package_compat(lio_scan_monitor_node nav_msgs)
target_link_ros2_package_compat(lio_scan_monitor_node sensor_msgs)
target_link_ros2_package_compat(lio_scan_monitor_node std_msgs)
target_link_ros2_package_compat(lio_scan_monitor_node geometry_msgs)
target_link_ros2_package_compat(lio_scan_monitor_node visualization_msgs)

install(
  TARGETS scan_analysis terminal_display lio_scan_monitor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_scan_analysis
    test/test_scan_analysis.cpp
  )
  target_compile_features(test_scan_analysis PUBLIC cxx_std_17)
  target_link_libraries(test_scan_analysis scan_analysis)
  add_test(NAME test_scan_analysis COMMAND test_scan_analysis)

  add_executable(test_terminal_display
    test/test_terminal_display.cpp
  )
  target_compile_features(test_terminal_display PUBLIC cxx_std_17)
  target_link_libraries(test_terminal_display terminal_display scan_analysis)
  add_test(NAME test_terminal_display COMMAND test_terminal_display)
endif()

ament_package()
