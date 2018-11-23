cmake_minimum_required(VERSION 2.8.3)
project(webcam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
find_package( OpenCV REQUIRED )
catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(webcam_node src/webcam.cpp)

target_link_libraries(webcam_node
  ${catkin_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBS}
)
