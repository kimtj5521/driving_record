cmake_minimum_required(VERSION 3.0.2)
project(driving_record)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


set( sources
  src/driving_record.cpp
)

add_executable(driving_record ${sources})
target_link_libraries(driving_record ${catkin_LIBRARIES})