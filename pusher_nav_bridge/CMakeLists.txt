cmake_minimum_required(VERSION 3.10)
project(pusher_nav_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/ros2_compat.cmake)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(pusher_nav_algorithms
  src/protocol.cpp
  src/route_processing.cpp
  src/side_distance_estimator.cpp
)
target_compile_features(pusher_nav_algorithms PUBLIC cxx_std_17)
target_include_directories(pusher_nav_algorithms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(pusher_nav_bridge_node
  src/pusher_nav_bridge_node.cpp
)
target_compile_features(pusher_nav_bridge_node PUBLIC cxx_std_17)
target_link_libraries(pusher_nav_bridge_node
  PRIVATE
    pusher_nav_algorithms
    rclcpp::rclcpp
)
target_link_ros2_package_compat(pusher_nav_bridge_node nav_msgs)
target_link_ros2_package_compat(pusher_nav_bridge_node sensor_msgs)

install(
  TARGETS pusher_nav_algorithms pusher_nav_bridge_node
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

ament_package()
