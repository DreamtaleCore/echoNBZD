cmake_minimum_required(VERSION 2.8.3)
project(mySocket)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES faceDetect
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

## Declare a cpp executable
 add_executable(myServer src/server.cpp)
target_link_libraries(myServer ${catkin_LIBRARIES} )

## Declare a cpp executable
 add_executable(myClient src/client.cpp)
target_link_libraries(myClient ${catkin_LIBRARIES} )

