QT       += core gui widgets network

TARGET = AHRSEmulator
TEMPLATE = app

INCLUDEPATH += ../common

SOURCES += main.cpp \
           vehicle.cpp \
           vehicle_rocket.cpp \
           emulator_window.cpp \
           ../common/gps_integrator.cpp

HEADERS  += vehicle.h \
            vehicle_rocket.h \
            emulator_window.h \
            emulator_defaults.h \
            ../common/gps_integrator.h

win32: LIBS += -luser32
