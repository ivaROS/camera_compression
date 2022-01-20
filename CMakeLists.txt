cmake_minimum_required(VERSION 2.8)

project(camera_compression)

find_package(catkin REQUIRED COMPONENTS
  roscpp
	cv_bridge
  sensor_msgs
  image_transport
  message_filters
  nav_msgs
  nodelet
  geometry_msgs
  std_msgs
  tf
  tf2
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES 
  CATKIN_DEPENDS
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(rgbd_throttle MODULE
	src/rgbd_throttle.cpp
)

add_library(depth_throttle MODULE
	src/depth_throttle.cpp
)

target_link_libraries(rgbd_throttle
	${catkin_LIBRARIES}
)

target_link_libraries(depth_throttle
	${catkin_LIBRARIES}
)

target_compile_options(rgbd_throttle PUBLIC -fopt-info-vec-optimized)

target_compile_options(depth_throttle PUBLIC -fopt-info-vec-optimized)

add_executable(image_processing_timing
  src/image_processing_timing.cpp
)

target_link_libraries(image_processing_timing
  ${catkin_LIBRARIES}
)
