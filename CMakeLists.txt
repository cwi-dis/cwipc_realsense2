# ======================================================
# Compiled by Fons @ CWI, Amsterdam for VRTogether
#
# Copyright (C) 2018 by CWI. All rights reserved.
# ======================================================
#
#  minimum required cmake version: 3.12.0
cmake_minimum_required(VERSION 3.12.0)

project(Capture-CWI)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# View the makefile commands during build
#set(CMAKE_VERBOSE_MAKEFILE on)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/CMakeFiles)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

option(BUILD_GRAPHICAL_EXAMPLES "Build graphical examples and tools." ON)

find_package(REALSENSE2 REQUIRED)
if(REALSENSE2_FOUND)
	include_directories(${REALSENSE2_INC})
	link_directories(${REALSENSE2_LIB})
else()
	message(FATAL_ERROR "\ncould not find realsense2 library\n")
endif()

find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(BUILD_GRAPHICAL_EXAMPLES)
    find_package(OpenGL)
    if(NOT OPENGL_FOUND)
        message(FATAL_ERROR "\n\n OpenGL package is missing!\n\n")
    endif()

	include_directories(D:/Fons/GitHub/librealsense/third-party/glfw/include;D:/Fons/GitHub/librealsense/third-party)

    list(APPEND DEPENDENCIES ${OPENGL_LIBRARIES})

    if(WIN32)
        list(APPEND DEPENDENCIES D:/Fons/GitHub/librealsense/build/third-party/glfw/Debug/glfw3.lib)
    else()
        # Find glfw header
        find_path(GLFW_INCLUDE_DIR NAMES GLFW/glfw3.h
            PATHS /usr/X11R6/include
                  /usr/include/X11
                  /opt/graphics/OpenGL/include
                  /opt/graphics/OpenGL/contrib/libglfw
                  /usr/local/include
                  /usr/include/GL
                  /usr/include
        )
        # Find glfw library
        find_library(GLFW_LIBRARIES NAMES glfw glfw3
                PATHS /usr/lib64
                      /usr/lib
                      /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
                      /usr/local/lib64
                      /usr/local/lib
                      /usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}
                      /usr/X11R6/lib
        )
        if(APPLE)
            find_library(COCOA_LIBRARY Cocoa)
            find_library(IOKIT_LIBRARY IOKit)
            find_library(COREVIDEO_LIBRARY CoreVideo)
            LIST(APPEND DEPENDENCIES ${COCOA_LIBRARY} ${IOKIT_LIBRARY} ${COREVIDEO_LIBRARY})
        endif()
        list(APPEND DEPENDENCIES m ${GLFW_LIBRARIES} ${LIBUSB1_LIBRARIES})
        include_directories(${GLFW_INCLUDE_DIR})
    endif()
else()
    if(NOT WIN32)
        list(APPEND DEPENDENCIES m ${LIBUSB1_LIBRARIES})
    endif()
endif()

add_subdirectory(src)
