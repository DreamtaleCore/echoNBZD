#-------------------------------------------------
#
# Project created by QtCreator 2016-03-06T14:41:01
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = videoStab
TEMPLATE = app


SOURCES += main.cpp\
        widget.cpp \
    welcom.cpp \
    about.cpp

HEADERS  += widget.h \
        cv4stab.h \
    welcom.h \
    about.h \
    myThread.h

FORMS    += widget.ui \
    welcom.ui \
    about.ui

RESOURCES   += source.qrc

INCLUDEPATH += /usr/local/include \
                /usr/local/include/opencv \
                /usr/local/include/opencv2

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
        /usr/local/lib/libopencv_calib3d.so
