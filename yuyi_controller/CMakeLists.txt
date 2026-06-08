cmake_minimum_required(VERSION 3.10)
project(yuyi_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/ros2_compat.cmake)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_executable(yuyi_controller_node
  src/yuyi_controller_node.cpp
)
target_compile_features(yuyi_controller_node PUBLIC cxx_std_17)
target_link_libraries(yuyi_controller_node
  PRIVATE
    ament_index_cpp::ament_index_cpp
    rclcpp::rclcpp
    yaml-cpp
)
target_link_ros2_package_compat(yuyi_controller_node geometry_msgs)
target_link_ros2_package_compat(yuyi_controller_node nav_msgs)
target_link_ros2_package_compat(yuyi_controller_node visualization_msgs)

install(
  TARGETS yuyi_controller_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
