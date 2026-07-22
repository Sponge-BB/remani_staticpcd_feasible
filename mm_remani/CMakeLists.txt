cmake_minimum_required(VERSION 3.0.2)
project(mm_remani)

find_package(catkin REQUIRED COMPONENTS
  livox_ros_driver2
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS livox_ros_driver2 roscpp rospy sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(livox_custom_to_pcl src/livox_custom_to_pcl.cpp)
add_dependencies(livox_custom_to_pcl ${catkin_EXPORTED_TARGETS})
target_link_libraries(livox_custom_to_pcl ${catkin_LIBRARIES})
