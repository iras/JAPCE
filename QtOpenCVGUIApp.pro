#-------------------------------------------------
#
# Project created by QtCreator 2012-05-25T21:38:18
#
#-------------------------------------------------

QT += core gui opengl

TARGET = QtOpenCVGUIApp
TEMPLATE = app

# the two QMAKE flags spare the system from getting an annoying "error: explicit instantiation of 'std::basic_ostream but no definition available"
QMAKE_CFLAGS_X86_64  += -mmacosx-version-min=10.8
QMAKE_CXXFLAGS_X86_64 = -mmacosx-version-min=10.8

SOURCES += main.cpp \
           mainwindow.cpp \
    	   reconstructor.cpp \
    	   glframe.cpp \
           glrenderthread.cpp \
           camerashot.cpp \
           pcsegment.cpp \
           pointcloud.cpp

HEADERS  += mainwindow.h\
            matcher.h \
            reconstructor.h \
            glframe.h \
            glrenderthread.h \
            camerashot.h \
            pcsegment.h \
            pointcloud.h

FORMS    += mainwindow.ui

#include (model/model.pri)

# OpenCV library
INCLUDEPATH += /opt/local/include/

LIBS += -L/opt/local/lib/ -lopencv_core \
                          -lopencv_highgui \
                          -lopencv_flann \
                          -lopencv_imgproc \
                          -lopencv_features2d \
                          -lopencv_calib3d \
                          -lopencv_nonfree \
                          -lopencv_legacy
#LIBS += -L/opt/local/lib/ -lQtOpenGL


# PCL library
INCLUDEPATH += /opt/local/include/eigen3 \
               /opt/local/include/flann/

#INCLUDEPATH += /opt/local/include/boost
LIBS += -L/opt/local/lib/ -lboost_system-mt

INCLUDEPATH +=  /usr/local/include/pcl-1.7/pcl/

LIBS += -L/usr/local/lib/ -lpcl_common \-lpcl_io \
                          -lpcl_sample_consensus \
                          -lpcl_registration \
                          -lpcl_segmentation \
                          -lpcl_features \
                          -lpcl_filters \
                          -lpcl_kdtree \
                          -lpcl_search


