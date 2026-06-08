cmake_minimum_required(VERSION 3.10)
project(simulation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/ros2_compat.cmake)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
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
    rclcpp::rclcpp
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_ros::tf2_ros
)
target_link_ros2_package_compat(ackermann_simulator_node geometry_msgs)
target_link_ros2_package_compat(ackermann_simulator_node nav_msgs)
target_link_ros2_package_compat(ackermann_simulator_node std_msgs)
target_link_ros2_package_compat(ackermann_simulator_node visualization_msgs)

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

if(BUILD_TESTING)
  enable_testing()
  add_test(
    NAME test_simulation_launch_defaults
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch_defaults.py
  )
  add_test(
    NAME test_simulation_path_generator_map_loading
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_path_generator_map_loading.py
  )
endif()

ament_package()
