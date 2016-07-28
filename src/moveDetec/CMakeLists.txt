cmake_minimum_required(VERSION 2.8.3)
project(moveDetec)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES moveDetec
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(detecMoving src/detecMoving.cpp)
target_link_libraries(detecMoving ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(detecCircle src/detecCircle.cpp)
target_link_libraries(detecCircle ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(detecFace src/faceDetec01.cpp)
target_link_libraries(detecFace ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(detecColor src/deteColor.cpp)
target_link_libraries(detecColor ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
