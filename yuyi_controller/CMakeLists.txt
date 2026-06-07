cmake_minimum_required(VERSION 3.10)
project(yuyi_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
target_include_directories(yuyi_controller_node PRIVATE
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)
target_link_libraries(yuyi_controller_node
  PRIVATE
    ament_index_cpp::ament_index_cpp
    ${geometry_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    rclcpp::rclcpp
    ${visualization_msgs_LIBRARIES}
    yaml-cpp
)

install(
  TARGETS yuyi_controller_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
