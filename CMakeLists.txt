cmake_minimum_required(VERSION 3.13)
project(LodViewer)

# Release as default
if (NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected, default to Release")
  set(CMAKE_BUILD_TYPE "Release")
endif()


if (CMAKE_COMPILER_IS_GNUCC)
  if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8.0)
    message(FATAL_ERROR "Currently running on v${CMAKE_CXX_COMPILER_VERSION}. Need at least gcc v8.0.0")
  endif()
endif()

if (NOT BUILD_SHARED_LIBS)
  message(STATUS "No shared/static libs type selected, default to static libs")
  set(BUILD_SHARED_LIBS OFF)
endif()

if (CMAKE_BUILD_TYPE MATCHES Debug)
  message("=====================> Debug Mode <=====================")
  set(CMAKE_CXX_FLAGS "-std=c++17 -O0 -mtune=generic -g -rdynamic -DDEBUG")
elseif (CMAKE_BUILD_TYPE MATCHES Release AND (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_GNUC))
  set(CMAKE_CXX_FLAGS "-Wall -std=c++17 -O3 -mtune=native -march=native -mssse3 -finline-functions -fgcse-sm -fgcse-las -ftracer -funroll-loops -ftree-vectorize -ftree-vectorizer-verbose=0 -DNDEBUG")
else()
  message(STATUS "Warning: Your compiler is not supported.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

################################################## THIRD PARTY {
include_directories(SYSTEM ThirdParty)
include_directories(SYSTEM ThirdParty/eigen-3.3.7)

# Since Eigen lib produces deprecated copies
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-copy -Wall -Wno-deprecated-declarations")

FIND_PACKAGE(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

# MKL
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(MKL REQUIRED)
set(EIGEN_USE_MKL_ALL 1)

# Gflags
find_package(gflags) 

# OpenGL
find_package(OpenGL REQUIRED)

# Qt
set(QT_LIB_REL_PATH "../../LIB/Qt")
if(UNIX AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${QT_LIB_REL_PATH}/5.15.2/gcc_64/lib/libQt5Core.so")
  include_directories(SYSTEM ${QT_LIB_REL_PATH}/5.15.2/gcc_64/include)
  include_directories(SYSTEM ${QT_LIB_REL_PATH}/5.15.2/gcc_64/include/QtWidgets)
  include_directories(SYSTEM ${QT_LIB_REL_PATH}/5.15.2/gcc_64/include/QtGui)
  include_directories(SYSTEM ${QT_LIB_REL_PATH}/5.15.2/gcc_64/include/QtCore)
  link_directories(${QT_LIB_REL_PATH}/5.15.2/gcc_64/lib)
  set(QT_LIBS
    Qt5Core
    Qt5Gui
    Qt5Widgets
  )
else()
  find_package(Qt5 COMPONENTS Widgets REQUIRED)
  find_package(Qt5 COMPONENTS Gui REQUIRED)
  set(QT_LIBS
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
  )
endif()

################################################## } THIRD PARTY

### for project source code, make sure we compile without warnings
### also disallow implicit conversions since they cause lots of bugs
if (CMAKE_BUILD_TYPE MATCHES Release AND (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_GNUC))
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wconversion -Wsign-conversion -Wextra -Werror")
endif()

# Set output folders
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

include_directories(src)
add_subdirectory(src)
