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
target_include_directories(ackermann_simulator_node PRIVATE
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)
target_link_libraries(ackermann_simulator_node
  PRIVATE
    ${geometry_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    rclcpp::rclcpp
    ${std_msgs_LIBRARIES}
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_ros::tf2_ros
    ${visualization_msgs_LIBRARIES}
)

install(
  TARGETS ackermann_simulator_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/nav2_map_publisher.py
    scripts/path_generator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch maps paths
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
