cmake_minimum_required(VERSION 3.0.2)
project(robotics_hackathon_automation)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
