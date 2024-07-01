cmake_minimum_required(VERSION 3.5)
project(my_robot_planner)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)  # IMU 메시지 패키지 추가

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# Add executables
add_executable(global_planner src/global_planner.cpp)
add_executable(planner_test src/planner_test.cpp)

# Link libraries
ament_target_dependencies(global_planner
  rclcpp
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  tf2_ros
  OpenCV
  yaml-cpp
)

ament_target_dependencies(planner_test
  rclcpp
  nav_msgs
  sensor_msgs  # IMU 메시지 패키지 추가
)

target_link_libraries(global_planner
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
)

# Install
install(TARGETS global_planner
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS planner_test
  DESTINATION lib/${PROJECT_NAME})

ament_package()
