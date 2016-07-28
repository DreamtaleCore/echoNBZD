QT += core
QT -= gui

CONFIG += c++11

TARGET = arucoProj
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

#SOURCES += /home/nbzd/bin/aruco-2.0.9/src/*.cpp \
#    aruco_tracker.cpp

SOURCES += \
    mymarker.cpp \
    client.cxx
    #camera_calibration.cpp

#HEADERS += /home/nbzd/bin/aruco-2.0.9/src/*.h

INCLUDEPATH += /opt/ros/indigo/include \
                /usr/local/include \
                /usr/local/include/eigen3 \
                /home/nbzd/bin/arduino-1.6.9/hardware/arduino/avr/cores/arduino \
                /home/nbzd/ws/src/Onboard-SDK-ROS-3.1/dji_sdk/include


LIBS += /usr/local/lib/*.so \
        /opt/ros/indigo/lib/*.so

