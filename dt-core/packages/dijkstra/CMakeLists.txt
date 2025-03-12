cmake_minimum_required(VERSION 2.8.3)
project(dijkstra VERSION 1.0.0)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  duckietown_msgs
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  duckietown_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)