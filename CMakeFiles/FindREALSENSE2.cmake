# ======================================================
# Compiled by Fons @ CWI, Amsterdam for VRTogether
#
# This code serves to find Intel RealSense SDK software
# Running it defines:
#	REALSENSE2_FOUND
#	REALSENSE2_INC
#	REALSENSE2_LIB
#	REALSENSE2_DLL
#
# Copyright (C) 2018 by CWI. All rights reserved.
# ======================================================

if(WIN32)

  set(CMAKE_FIND_LIBRARY_SUFFIXES .dll ${CMAKE_FIND_LIBRARY_SUFFIXES})

endif()


if(MSVC)
  list(APPEND REALSENSE2_INCLUDE_DIRS $ENV{REALSENSE2_HOME}/include)
  list(APPEND REALSENSE2_LIBRARY_DIRS $ENV{REALSENSE2_HOME}/lib)
  list(APPEND REALSENSE2_DLL_PATHS $ENV{REALSENSE2_HOME}/bin)
  list(APPEND REALSENSE2_INCLUDE_DIRS $ENV{REALSENSE2_DIR}/include)
  list(APPEND REALSENSE2_LIBRARY_DIRS $ENV{REALSENSE2_DIR}/lib)
  list(APPEND REALSENSE2_DLL_PATHS $ENV{REALSENSE2_DIR}/bin)

  if(CMAKE_CL_64)
    list(APPEND REALSENSE2_INCLUDE_DIRS "C:/Program Files (x86)/Intel RealSense SDK 2.0/include")
    list(APPEND REALSENSE2_LIBRARY_DIRS "C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/x64")
    list(APPEND REALSENSE2_DLL_PATHS "C:/Program Files (x86)/Intel RealSense SDK 2.0/bin/x64")
  else()
    list(APPEND REALSENSE2_INCLUDE_DIRS "C:/Program Files (x86)/Intel RealSense SDK 2.0/include")
    list(APPEND REALSENSE2_LIBRARY_DIRS "C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/x86")
    list(APPEND REALSENSE2_DLL_PATHS "C:/Program Files (x86)/Intel RealSense SDK 2.0/bin/x86")
  endif()

else()
  list(APPEND REALSENSE2_INCLUDE_DIRS /usr/include)
  list(APPEND REALSENSE2_LIBRARY_DIRS /usr/lib)
  list(APPEND REALSENSE2_DLL_PATHS /usr/bin)
  list(APPEND REALSENSE2_INCLUDE_DIRS $ENV{REALSENSE2_HOME}/include)
  list(APPEND REALSENSE2_LIBRARY_DIRS $ENV{REALSENSE2_HOME}/lib)
  list(APPEND REALSENSE2_DLL_PATHS $ENV{REALSENSE2_HOME}/bin)
  list(APPEND REALSENSE2_INCLUDE_DIRS $ENV{REALSENSE2_DIR}/include)
  list(APPEND REALSENSE2_LIBRARY_DIRS $ENV{REALSENSE2_DIR}/lib)
  list(APPEND REALSENSE2_DLL_PATHS $ENV{REALSENSE2_DIR}/bin)
endif()

find_path(REALSENSE2_INC librealsense2/rs.hpp PATHS ${REALSENSE2_INCLUDE_DIRS})
find_library(REALSENSE2_LIB NAMES realsense2 PATHS ${REALSENSE2_LIBRARY_DIRS})
find_library(REALSENSE2_DLL NAMES realsense2 PATHS ${REALSENSE2_DLL_PATHS})

if(REALSENSE2_LIB AND REALSENSE2_INC AND REALSENSE2_DLL)
  set(REALSENSE2_FOUND TRUE)
else()
  set(REALSENSE2_FOUND FALSE)
endif()
