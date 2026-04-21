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
find_package(Qt6 REQUIRED COMPONENTS OpenGL)
find_package(Qt6 REQUIRED COMPONENTS OpenGLWidgets)

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
  triangle.h
  rgba.h
  monster.h monster.cpp
  triangle.c


  glwidget.cpp
  graphics/camera.cpp
  graphics/graphicsdebug.cpp
  graphics/meshloader.cpp
  graphics/shader.cpp
  graphics/shape.cpp

  glwidget.h
  graphics/camera.h
  graphics/graphicsdebug.h
  graphics/meshloader.h
  graphics/shader.h
  graphics/shape.h

  util/tiny_obj_loader.h
  util/unsupportedeigenthing/OpenGLSupport

  resources/shaders/shaders.qrc
)

# GLEW: this creates its library and allows you to `#include "GL/glew.h"`
add_library(StaticGLEW STATIC glew/src/glew.c)
include_directories(${PROJECT_NAME} PRIVATE glew/include)

set_source_files_properties(triangle.c PROPERTIES
    LANGUAGE C
    COMPILE_FLAGS "-DTRILIBRARY -DANSI_DECLARATORS -DREAL=double -DVOID=int")

# Specifies libraries to be linked (Qt components, glew, etc)

include(FetchContent)

# libigl still uses the deprecated FetchContent_Populate() API internally.
# Silence the CMake 3.30 deprecation warning by forcing OLD behavior for CMP0169.
if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)
    set(CMAKE_POLICY_DEFAULT_CMP0169 OLD)
endif()

FetchContent_Declare(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG main
)

FetchContent_MakeAvailable(libigl)

target_link_libraries(${PROJECT_NAME} PRIVATE
  StaticGLEW
  Qt::Core
  Qt::Widgets
  Qt::Gui
  Qt::OpenGL
  Qt::OpenGLWidgets
  igl::core
  # CDT::CDT
)

# GLEW on macOS needs to link against the system OpenGL framework.
if (APPLE)
  find_package(OpenGL REQUIRED)
  target_link_libraries(${PROJECT_NAME} PRIVATE OpenGL::GL)
endif()

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

