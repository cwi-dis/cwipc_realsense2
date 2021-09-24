# ======================================================
# Compiled by Fons @ CWI, Amsterdam for VRTogether
#
# Copyright (C) 2018 by CWI. All rights reserved.
# ======================================================
#
#  minimum required cmake version: 3.10.0
cmake_minimum_required(VERSION 3.10.0)

project(cwipc_realsense2)


list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/CMakeFiles)

option(BUILD_SHARED_LIBS "Build shared library" ON)

include(CTest)
include(GNUInstallDirs)

# Ensure all executable, dll/so and link-library outputs end up in the same directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})

#
# Find cwipc_util
#
if( CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR )
	find_package(cwipc_util REQUIRED)
	include_directories(${cwipc_util_INCLUDE_DIRS})
	link_directories(${cwipc_util_LIBRARY_DIRS})
endif()
#
# Find PCL. 1.11 has been tested.
#
find_package(PCL 1.11 REQUIRED COMPONENTS common io filters) 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#
# Find realsense (either with pkgconfig or with cmake)
#
find_package(REALSENSE2 REQUIRED)
include_directories(${REALSENSE2_INCLUDE_DIRS})
link_directories(${REALSENSE2_LIBRARY_DIRS})

# we need the thirdparty tinyXML for the test programs.
include(third-party/tinyXML/CMakeLists.txt)
include_directories(third-party/tinyXML)

# If we're on Linux os MacOS we need libusb

if(NOT WIN32)
	list(APPEND DEPENDENCIES m ${LIBUSB1_LIBRARIES})
endif()

include_directories(include)

add_subdirectory(src)
add_subdirectory(apps/cwipc_rs2grab)
add_subdirectory(apps/cwipc_rs2multigrab)
add_subdirectory(apps/cwipc_rs2offline)
add_subdirectory(python)
