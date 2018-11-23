cmake_minimum_required(VERSION 2.8.3)
project(readmarker)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)
find_package( OpenCV REQUIRED )

add_message_files(FILES markermsg.msg markerXY.msg)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  LIBRARIES readmarker
  CATKIN_DEPENDS std_msgs roscpp
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(readmarker_node src/readmarker.cpp)
add_dependencies(readmarker_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})
target_link_libraries(readmarker_node ${catkin_LIBRARIES})


target_link_libraries(readmarker_node
  ${catkin_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBS}
)
