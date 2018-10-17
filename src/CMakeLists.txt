# ======================================================
# Compiled by Fons @ CWI, Amsterdam for VRTogether
#
# Copyright (C) 2018 by CWI. All rights reserved.
# ======================================================
#
#  minimum required cmake version: 3.12.0
cmake_minimum_required(VERSION 3.12.0)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX D:/${PROJECT_NAME} CACHE PATH "..." FORCE)
endif()

add_library(multiFrame SHARED multiFrame.cpp multiFrame.hpp ${TINY_SRC} )
target_link_libraries(multiFrame ${REALSENSE2_LIB})
