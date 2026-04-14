cmake_minimum_required(VERSION 3.16)

# Sets project name
project(projects_raster LANGUAGES CXX C)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Sets C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Specifies required Qt components
find_package(Qt6 REQUIRED COMPONENTS Core)
find_package(Qt6 REQUIRED COMPONENTS Widgets)
find_package(Qt6 REQUIRED COMPONENTS Gui)

# Specifies required Qt components
add_definitions(-D_USE_MATH_DEFINES)
add_definitions(-DTIXML_USE_STL)

# Specifies .cpp and .h files to be passed to the compiler
add_executable(${PROJECT_NAME}
  main.cpp

  mainwindow.cpp
  settings.cpp
  canvas2d.cpp

  mainwindow.h
  settings.h
  canvas2d.h
  rgba.h
  monster.h monster.cpp
  triangle.c
)

set_source_files_properties(triangle.c PROPERTIES
    LANGUAGE C
    COMPILE_FLAGS "-DTRILIBRARY -DANSI_DECLARATORS -DREAL=double -DVOID=int")

# Specifies libraries to be linked (Qt components, glew, etc)

include(FetchContent)
FetchContent_Declare(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG main
)

FetchContent_MakeAvailable(libigl)

target_link_libraries(${PROJECT_NAME} PRIVATE
  Qt::Core
  Qt::Widgets
  Qt::Gui
  igl::core
  # CDT::CDT
)

# This allows you to `#include "Eigen/..."`
target_include_directories(${PROJECT_NAME} PRIVATE
    Eigen
    ${libigl_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}
)

# Set this flag to silence warnings on Windows
if (MSVC OR MSYS OR MINGW)
  set(CMAKE_CXX_FLAGS "-Wno-volatile")
endif()

# Set this flag to silence warnings on MacOS
if (APPLE)
  set(CMAKE_CXX_FLAGS "-Wno-deprecated-volatile")
endif()

