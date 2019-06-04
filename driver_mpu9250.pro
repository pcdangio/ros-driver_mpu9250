TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    src/driver.cpp \
    src/main_rpi.cpp \
    src/ros_node.cpp \
    src/rpi_driver.cpp

DISTFILES += \
    CMakeLists.txt \
    README.md \
    package.xml

HEADERS += \
    src/driver.h \
    src/ros_node.h \
    src/rpi_driver.h

INCLUDEPATH += \
    /opt/ros/melodic/include \
