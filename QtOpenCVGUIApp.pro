#-------------------------------------------------
#
# Project created by QtCreator 2012-05-25T21:38:18
#
#-------------------------------------------------

QT += core gui opengl

TARGET = QtOpenCVGUIApp
TEMPLATE = app


SOURCES += main.cpp\
           mainwindow.cpp \
    	   reconstructor.cpp \
    	   glframe.cpp \
           glrenderthread.cpp \
    mpointcloud.cpp

HEADERS  += mainwindow.h\
            matcher.h \
            reconstructor.h \
            glframe.h \
            glrenderthread.h \
    mpointcloud.h

FORMS    += mainwindow.ui

INCLUDEPATH += /opt/local/include/
LIBS += -L/opt/local/lib/ -lopencv_core -lopencv_highgui -lopencv_flann -lopencv_imgproc -lopencv_features2d -lopencv_calib3d -lopencv_nonfree -lopencv_legacy
#LIBS += -L/opt/local/lib/ -lQtOpenGL
