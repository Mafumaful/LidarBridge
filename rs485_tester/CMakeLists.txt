cmake_minimum_required(VERSION 3.8)
project(rs485_tester)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(rs485_test_node
  src/rs485_port.cpp
  src/rs485_test_node.cpp
)

target_compile_features(rs485_test_node PUBLIC cxx_std_17)
target_include_directories(rs485_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rs485_test_node rclcpp)

install(
  TARGETS rs485_test_node
  DESTINATION lib/${PROJECT_NAME}
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
