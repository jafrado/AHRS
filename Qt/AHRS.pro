QT       += core gui widgets serialport

TARGET = AHRS
TEMPLATE = app

SOURCES += main.cpp ahrs_widgets.cpp ahrs_window.cpp AHRS.cpp

HEADERS  += AHRS.h ahrs_widgets.h ahrs_window.h
FORMS    += ahrs_window.ui

