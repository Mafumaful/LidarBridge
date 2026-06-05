cmake_minimum_required(VERSION 3.10)
project(simulation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(ackermann_simulator_node
  src/ackermann_simulator_node.cpp
)
target_compile_features(ackermann_simulator_node PUBLIC cxx_std_17)
target_link_libraries(ackermann_simulator_node
  PRIVATE
    geometry_msgs::geometry_msgs
    nav_msgs::nav_msgs
    rclcpp::rclcpp
    std_msgs::std_msgs
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_ros::tf2_ros
    visualization_msgs::visualization_msgs
)

install(
  TARGETS ackermann_simulator_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
