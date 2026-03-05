cmake_minimum_required(VERSION 3.15)
project(AHRSEmulator)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

find_package(Qt5 CONFIG REQUIRED COMPONENTS Widgets SerialPort)

add_executable(${PROJECT_NAME} WIN32
    main.cpp
    emulator_window.h
    emulator_window.cpp
)

target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::SerialPort)
qt5_use_modules(${PROJECT_NAME} Widgets SerialPort)
