cmake_minimum_required(VERSION 2.8.3)
project(stableVideo)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED core ocl)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(stable src/stable.cxx) 
target_link_libraries(stable ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} opencv)
