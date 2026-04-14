cmake_minimum_required(VERSION 3.8)
project(sweep_mock_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(mock_robot_node
  src/mock_robot.cpp
)

ament_target_dependencies(mock_robot_node
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
  mock_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
