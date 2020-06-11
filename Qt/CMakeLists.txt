cmake_minimum_required(VERSION 3.15)

project (AHRS)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if (MSVC)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()
add_definitions( -DHAVE_QT)

# Find the QtWidgets library
find_package(Qt5 CONFIG REQUIRED Widgets)

set(project_SOURCES main.cpp ahrs_widgets.cpp ahrs_window.cpp AHRS.cpp)

add_executable(${PROJECT_NAME} ${project_SOURCES})
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
qt5_use_modules(${PROJECT_NAME} Widgets)
