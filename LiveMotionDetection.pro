#-------------------------------------------------
#
# Project created by QtCreator 2015-05-04T11:05:20
#
#-------------------------------------------------

QT       += core gui \
            network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = LiveMotionDetection
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
        qcustomplot.cpp

HEADERS  += mainwindow.h \
         qcustomplot.h \
         Vector_3D.h

FORMS    += mainwindow.ui
