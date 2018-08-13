TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

#INCLUDEPATH += /usr/include/gazebo-9
#INCLUDEPATH += /usr/include/ignition/math4
#INCLUDEPATH += /usr/include/ignition/msgs1
#INCLUDEPATH += /usr/include/sdformat-6.0

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv
