QT       += core gui widgets serialport network opengl

TARGET = AHRS
TEMPLATE = app

INCLUDEPATH += ../common

SOURCES += main.cpp ahrs_widgets.cpp ahrs_window.cpp AHRS.cpp appearance_dialog.cpp MASI.cpp \
           ../common/gps_integrator.cpp

HEADERS  += AHRS.h ahrs_widgets.h ahrs_window.h appearance_dialog.h MASI.h \
            ../common/gps_integrator.h
FORMS    += ahrs_window.ui

