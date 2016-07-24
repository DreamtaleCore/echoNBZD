QT += core
QT -= gui

CONFIG += c++11

TARGET = fireFu
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    commCtrl.cxx \
    arduinocomm.cpp

INCLUDEPATH += /opt/ros/indigo/include \
                /usr/local/include \
                /home/nbzd/bin/arduino-1.6.9/hardware/arduino/avr/cores/arduino

LIBS += /usr/local/lib/*.so \
        /opt/ros/indigo/lib/*.so

HEADERS += \
    sysHeaders.h \
    stdHeaders.h \
    commCtrl.h \
    arduinoHeaders.h
