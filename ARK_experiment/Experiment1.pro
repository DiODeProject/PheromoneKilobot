#-------------------------------------------------
#
# Project created by QtCreator 2018-01-01T07:35:48
#
#-------------------------------------------------

QT       -= gui

QT += widgets

TARGET = Experiment1
TEMPLATE = lib

DEFINES += EXPERIMENT1EXP_LIBRARY

SOURCES += \
    kilobot.cpp \
    Experiment1Env.cpp \
    Experiment1Exp.cpp

HEADERS +=\
    kilobot.h \
    kilobotexperiment.h \
    kilobotenvironment.h \
    global.h \
    Experiment1Env.h \
    Experiment1Exp.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

INCLUDEPATH += /usr/local/include/
LIBS += -L/usr/local/lib \
        -lopencv_core
