#-------------------------------------------------
#
# Project created by QtCreator 2016-03-16T11:30:48
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = forUAV
CONFIG   += console
CONFIG   -= app_bundle

HEADERS += opencvHeaders.h              \
        aruco_lib/aruco.h               \
        aruco_lib/arucofidmarkers.cpp   \
        aruco_lib/arucofidmarkers.h     \
        aruco_lib/board.cpp             \
        aruco_lib/board.h               \
        aruco_lib/boarddetector.cpp     \
        aruco_lib/boarddetector.h       \
        aruco_lib/cameraparameters.cpp  \
        aruco_lib/cameraparameters.h    \
        aruco_lib/cvdrawingutils.cpp    \
        aruco_lib/cvdrawingutils.h      \
        aruco_lib/exports.h             \
        aruco_lib/marker.cpp            \
        aruco_lib/marker.h              \
        aruco_lib/markerdetector.cpp    \
        aruco_lib/markerdetector.h \
    arucoHeaders.h \
    stdHeaders.h   \

TEMPLATE = app

SOURCES += main.cpp \
    stableVideo.cxx

INCLUDEPATH += /usr/local/include \
                /usr/local/include/opencv \
                        /usr/local/include/opencv2 \
        /opt/ros/indigo/include

LIBS += /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_legacy.so \
        /usr/local/lib/libopencv_ml.so \
        /usr/local/lib/libopencv_core.so \
        /usr/local/lib/libopencv_imgproc.so \
        /usr/local/lib/libopencv_nonfree.so \
        /usr/local/lib/libopencv_gpu.so \
        /usr/local/lib/libopencv_core.so \
        /usr/local/lib/libopencv_objdetect.so \
        /usr/local/lib/libopencv_features2d.so \
        /usr/local/lib/libopencv_contrib.so \
        /usr/local/lib/libopencv_video.so \
        /usr/local/lib/libopencv_superres.so \
        /usr/local/lib/libopencv_stitching.so \
        /usr/local/lib/libopencv_photo.so \
        /usr/local/lib/libopencv_ocl.so \
        /usr/local/lib/libopencv_flann.so.2.4 \
        /usr/local/lib/libopencv_calib3d.so \
        /usr/local/lib/libaruco.so \
        /usr/local/lib/libopencv_calib3d.so \

