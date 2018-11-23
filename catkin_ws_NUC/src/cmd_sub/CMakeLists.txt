cmake_minimum_required(VERSION 2.8.3)
project(cmd_sub)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(cmd_sub_node src/cmd_sub.cpp)

target_link_libraries(cmd_sub_node
   ${catkin_LIBRARIES}
)
