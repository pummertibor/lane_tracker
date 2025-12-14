cmake_minimum_required(VERSION 3.8)
project(lane_tracker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# Nem kell gazebo_ros, mert ros_gz_bridge-et használunk launch-ban

add_executable(object_avoider src/object_avoider.cpp)
ament_target_dependencies(object_avoider rclcpp geometry_msgs sensor_msgs nav_msgs)

install(TARGETS object_avoider DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch simulation DESTINATION share/${PROJECT_NAME})

ament_package()