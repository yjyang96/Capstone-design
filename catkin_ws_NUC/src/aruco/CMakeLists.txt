cmake_minimum_required(VERSION 2.8.3)
project(aruco)

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

add_executable(aruco_node src/aruco.cpp)

target_link_libraries(aruco_node
  ${catkin_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBS}
)
