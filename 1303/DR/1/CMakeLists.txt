cmake_minimum_required(VERSION 2.8.3)
project(talker)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs turtlesim)

## Generate added messages and services
generate_messages(DEPENDENCIES geometry_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(tlkr src/talker.cpp)
target_link_libraries(tlkr ${catkin_LIBRARIES})
