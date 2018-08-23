cmake_minimum_required(VERSION 2.8.3)
project(ibeo_scala)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
)
find_package(PCL 1.7 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
        FILES
  Point2D.msg
  ObjectArray.msg
  Object.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS
  message_runtime
	std_msgs
)

include_directories(
		${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  /home/guolindong/CyberC3/IbeoSDK5.2.2/src
)

## Declare a C++ library
add_library(ibeosdk_d SHARED IMPORTED)
set_target_properties(ibeosdk_d PROPERTIES IMPORTED_LOCATION /home/guolindong/CyberC3/IbeoSDK5.2.2/libs/libibeosdk_d.so)

add_executable(scala_node
        src/scala_pointcloud.cpp
)

target_link_libraries(scala_node
        ibeosdk_d
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
 )

