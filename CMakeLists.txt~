cmake_minimum_required(VERSION 2.8.3)
project(ibeo_scala_node)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
)
find_package(PCL REQUIRED)

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
# include
  ${catkin_INCLUDE_DIRS}
  /home/guolindong/CyberC3/IbeoSDK5.2.2/src
)

## Declare a C++ library
add_library(ibeosdk_d SHARED IMPORTED)
set_target_properties(ibeosdk_d PROPERTIES IMPORTED_LOCATION /home/guolindong/CyberC3/IbeoSDK5.2.2/libs/libibeosdk_d.so)

add_executable(scala_pointcloud
        src/scala_pointcloud.cpp
)

#link_directories(
#        ${catkin_LIB_DIRS}
#        /home/guolindong/CyberC3/IbeoSDK5.2.2/libs/
#)

## Specify libraries to link a library or executable target against
target_link_libraries(scala_pointcloud
        ibeosdk_d
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
 )

