cmake_minimum_required(VERSION 3.8)

project(wachroboter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(
  wachroboter_node
  src/wachroboter_node.cpp
)

ament_target_dependencies(
  wachroboter_node
  rclcpp
  rclcpp_action
  nav2_msgs
  std_msgs
)

install(
  TARGETS wachroboter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
