cmake_minimum_required(VERSION 2.8.3)
project(deadreckoning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  # duckietown_msgs # Every duckietown packages should use this.
  message_generation
)

add_service_files(
  FILES
  SetPoint.srv
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
