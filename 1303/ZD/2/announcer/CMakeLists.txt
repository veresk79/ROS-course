cmake_minimum_required(VERSION 2.8.3)
project(announcer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(announcer src/announcer.cpp)
add_dependencies(announcer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(announcer ${catkin_LIBRARIES})

