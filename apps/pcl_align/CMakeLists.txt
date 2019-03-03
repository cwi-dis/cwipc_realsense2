# ======================================================
# Compiled by Fons @ CWI, Amsterdam for VRTogether
#
# Copyright (C) 2018 by CWI. All rights reserved.
# ======================================================
#
#  minimum required cmake version: 3.10.0
cmake_minimum_required(VERSION 3.10.0)

set(DEPENDENCIES cwipc_realsense2)
include_directories(../../include)
#
# Find glfw (or use slurped version)
#
if(PKG_CONFIG_FOUND)
	pkg_check_modules(GLFW glfw3)
	if(GLFW_FOUND)
		include_directories(${GLFW_INCLUDE_DIRS})
		link_directories(${GLFW_LIBRARY_DIRS})
		add_definitions(${GLFW_DEFINITIONS})
		list(APPEND DEPENDENCIES ${GLFW_LIBRARIES})
	endif()
endif()
if(NOT GLFW_FOUND)
	list(APPEND DEPENDENCIES glfw3)
endif()

#
# Find gl libraries
#
if(PKG_CONFIG_FOUND)
	pkg_check_modules(OPENGL glu gl)
	if(OPENGL_FOUND)
		add_definitions(-D_GLFW_USE_OPENGL -D_GLFW_X11)
	endif()
endif()
if(NOT OPENGL_FOUND)
	find_package(OpenGL REQUIRED)
endif()
list(APPEND DEPENDENCIES ${OPENGL_LIBRARIES})

# Find Boost
find_package(Boost COMPONENTS system REQUIRED)

add_executable(pcl_align pcl_align.cpp  window_util.cpp)

target_link_libraries(pcl_align ${DEPENDENCIES} ${Boost_LIBRARIES})
install(TARGETS pcl_align RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
