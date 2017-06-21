cmake_minimum_required(VERSION 2.8)

project(camera_compression)

find_package(catkin REQUIRED COMPONENTS
  roscpp
	cv_bridge
  sensor_msgs
  image_transport
  message_filters
  nodelet
  
)

catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(rgbd_throttle MODULE
	src/rgbd_throttle.cpp
)

target_link_libraries(rgbd_throttle
	${catkin_LIBRARIES}
)
