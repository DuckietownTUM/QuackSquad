cmake_minimum_required(VERSION 2.8.3)
project(deadreckoning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  duckietown_msgs
)

catkin_python_setup()

add_service_files(
  FILES
  SetPoint.srv
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
