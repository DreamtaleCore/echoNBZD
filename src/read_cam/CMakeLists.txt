cmake_minimum_required(VERSION 2.8.3)
project(read_cam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp std_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV: ${OpenCV_LIBS}")

#This command helped me to solve the problem:
#pkg-config libusb-1.0 --libs --cflags --variable=libdir --print-variables
#pkg-config opencv --cflags --libs

## code to find libusb-1.0 library
FIND_PATH(LIBUSB_1_INCLUDE_DIR libusb.h
   /usr/include/libusb-1.0
)
message(STATUS "Found libusb.h: ${LIBUSB_1_INCLUDE_DIR}")

FIND_LIBRARY(LIBUSB_1_LIBRARY NAMES usb-1.0 
   /usr/lib/x86_64-linux-gnu
)
message(STATUS "Found libusb 1.0 library: ${LIBUSB_1_LIBRARY}")

if (LIBUSB_1_INCLUDE_DIR AND LIBUSB_1_LIBRARY)
   set(LIBUSB_1_FOUND TRUE)
   set(LIBUSB_1_LIBRARIES ${LIBUSB_1_LIBRARY})
else (LIBUSB_1_INCLUDE_DIR AND LIBUSB_1_LIBRARY)
   set(LIBUSB_1_FOUND FALSE)
endif (LIBUSB_1_INCLUDE_DIR AND LIBUSB_1_LIBRARY)

if (LIBUSB_1_FOUND)
      message(STATUS "Found LibLIBUSB_1: ${LIBUSB_1_LIBRARIES}")
else (LIBUSB_1_FOUND)
	   message(STATUS "don't find LibLIBUSB_1")
endif (LIBUSB_1_FOUND)

MARK_AS_ADVANCED(LIBUSB_1_INCLUDE_DIR LIBUSB_1_LIBRARIES LIBUSB_1_LIBRARY)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES read_cam
#  CATKIN_DEPENDS cv_bridge image_transport opencv2 roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(webCameraRead src/webCameraRead.cpp)


target_link_libraries(webCameraRead
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)
