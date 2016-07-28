cmake_minimum_required(VERSION 2.8.3)
project(posiCvt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(posiCvt src/pixel2angle.cpp)
target_link_libraries(posiCvt ${catkin_LIBRARIES})

add_executable(posiCvt4Color src/pixel2angle4color.cpp)
target_link_libraries(posiCvt4Color ${catkin_LIBRARIES})
