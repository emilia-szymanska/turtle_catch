cmake_minimum_required(VERSION 3.0.2)
project(turtle_catch)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  turtlesim
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

