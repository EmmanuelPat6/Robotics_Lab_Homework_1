cmake_minimum_required(VERSION 3.8)
project(arm_control)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(arm_controller src/arm_controller_node.cpp)
ament_target_dependencies(arm_controller
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(publisher_terminal src/Other_Publishers/arm_controller_node_terminal.cpp)
ament_target_dependencies(publisher_terminal
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(publisher_defined src/Other_Publishers/arm_controller_node_defined.cpp)
ament_target_dependencies(publisher_defined
  rclcpp
  sensor_msgs
  std_msgs
)


  
install(
  TARGETS arm_controller publisher_terminal publisher_defined
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
