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
option(BUILD_SHARED_LIBS "Build shared library" ON)

# First try locating things with pkg-config.

find_package(PkgConfig)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(REALSENSE2 realsense2)
    if(REALSENSE2_FOUND)
        include_directories(${REALSENSE2_INCLUDE_DIRS})
        link_directories(${REALSENSE2_LIBRARY_DIRS})
    endif()
    pkg_check_modules(PCL pcl_common-1.7 pcl_io-1.7)
    if(PCL_FOUND)
        include_directories(${PCL_INCLUDE_DIRS})
        link_directories(${PCL_LIBRARY_DIRS})
        add_definitions(${PCL_DEFINITIONS})
    endif()
    pkg_check_modules(OPENGL glu gl)
    if(OPENGL_FOUND)
        add_definitions(-D_GLFW_USE_OPENGL -D_GLFW_X11)
    endif()
endif()

# if(NOT REALSENSE2_FOUND)
#     find_package(REALSENSE2 REQUIRED)
#     if(REALSENSE2_FOUND)
#         include_directories(${REALSENSE2_INC})
#         link_directories(${REALSENSE2_LIB})
#     else()
#         message(FATAL_ERROR "\ncould not find realsense2 library\n")
#     endif()
# endif()
# 
# if(NOT PCL_FOUND)
#     find_package(PCL REQUIRED COMPONENTS common io)
#     include_directories(${PCL_INCLUDE_DIRS})
#     link_directories(${PCL_LIBRARY_DIRS})
#     add_definitions(${PCL_DEFINITIONS})
# endif()

include(third-party/tinyXML/CMakeLists.txt)
include_directories(third-party/tinyXML)

if(BUILD_GRAPHICAL_EXAMPLES)

    add_subdirectory(third-party/glfw)
	list(APPEND DEPENDENCIES glfw3)
 
    find_package(OpenGL)
    if(NOT OPENGL_FOUND)
        message(FATAL_ERROR "\n\n OpenGL package is missing!\n\n")
    endif()

    list(APPEND DEPENDENCIES ${OPENGL_LIBRARIES})

else()
    if(NOT WIN32)
        list(APPEND DEPENDENCIES m ${LIBUSB1_LIBRARIES})
    endif()
endif()

add_subdirectory(src)
