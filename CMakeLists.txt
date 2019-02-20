# ======================================================
# Compiled by Fons @ CWI, Amsterdam for VRTogether
#
# Copyright (C) 2018 by CWI. All rights reserved.
# ======================================================
#
#  minimum required cmake version: 3.10.0
cmake_minimum_required(VERSION 3.10.0)

project(cwipc_realsense2)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# View the makefile commands during build
set(CMAKE_VERBOSE_MAKEFILE on)

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

include(GNUInstallDirs)

#
# Find cwipc_util
#
find_package(cwipc_util REQUIRED)
include_directories(${cwipc_util_INCLUDE_DIRS})
link_directories(${cwipc_util_LIBRARY_DIRS})

#
# Find realsense (either with pkgconfig or with cmake)
#
find_package(REALSENSE2 REQUIRED)
include_directories(${REALSENSE2_INCLUDE_DIRS})
link_directories(${REALSENSE2_LIBRARY_DIRS})

#
# Find PCL. Either 1.7 or 1.8 using pkgconfig, or using cmake finder.
#
find_package(PCL REQUIRED COMPONENTS common io visualization filters) 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# we need the thirdparty tinyXML for the test programs.
include(third-party/tinyXML/CMakeLists.txt)
include_directories(third-party/tinyXML)

# If we're on Linux os MacOS we need libusb

if(NOT WIN32)
	list(APPEND DEPENDENCIES m ${LIBUSB1_LIBRARIES})
endif()


if(BUILD_GRAPHICAL_EXAMPLES)
	if(PKG_CONFIG_FOUND)
		pkg_check_modules(GLFW glfw3)
	endif()
	if(NOT GLFW_FOUND)
		add_subdirectory(third-party/glfw)
	endif()
	
	add_subdirectory(renderer)
endif()

include_directories(include)

add_subdirectory(src)
