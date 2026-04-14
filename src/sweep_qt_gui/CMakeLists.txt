cmake_minimum_required(VERSION 3.8)
project(sweep_qt_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)
find_package(PCL REQUIRED COMPONENTS common io visualization)
find_package(VTK REQUIRED)

add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}
  include/sweep_qt_gui/mainwindow.h
  include/sweep_qt_gui/ros2_manager.h
  src/main.cpp
  src/mainwindow.cpp
  src/ros2_manager.cpp
  src/routelistdialog.cpp  
  include/sweep_qt_gui/routelistdialog.h
  ui/mainwindow.ui
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
