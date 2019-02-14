# ======================================================
# Compiled by Fons @ CWI, Amsterdam for VRTogether
#
# Copyright (C) 2018 by CWI. All rights reserved.
# ======================================================
#
#  minimum required cmake version: 3.10.0
cmake_minimum_required(VERSION 3.10.0)

set(DEPENDENCIES cwipc_realsense2)
include_directories(../include)
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

# Find Boost (only for pcl_align)
find_package(Boost COMPONENTS system REQUIRED)

add_executable(pcl_renderer pcl_renderer.cpp pcl_renderer.hpp window_util.hpp)
set_target_properties(pcl_renderer PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/lib/Debug"
	ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/lib/Release"
	ARCHIVE_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_BINARY_DIR}/lib/MinSizeRel"
	ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/lib/RelWithDebInfo"
	LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/lib/Debug"
	LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/lib/Release"
	LIBRARY_OUTPUT_DIRECTORY_MINSIZERELG "${CMAKE_BINARY_DIR}/lib/MinSizeRel"
	LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/lib/RelWithDebInfo"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin/Debug"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin/Release"
	RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_BINARY_DIR}/bin/MinSizeRel"
	RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/bin/RelWithDebInfo"
)
target_link_libraries(pcl_renderer ${DEPENDENCIES})
install(TARGETS pcl_renderer RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(pcl_align pcl_align.cpp pcl_align.hpp window_util.hpp)
set_target_properties(pcl_align PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/lib/Debug"
	ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/lib/Release"
	ARCHIVE_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_BINARY_DIR}/lib/MinSizeRel"
	ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/lib/RelWithDebInfo"
	LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/lib/Debug"
	LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/lib/Release"
	LIBRARY_OUTPUT_DIRECTORY_MINSIZERELG "${CMAKE_BINARY_DIR}/lib/MinSizeRel"
	LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/lib/RelWithDebInfo"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin/Debug"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin/Release"
	RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_BINARY_DIR}/bin/MinSizeRel"
	RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/bin/RelWithDebInfo"
)

target_link_libraries(pcl_align ${DEPENDENCIES} ${Boost_LIBRARIES})
install(TARGETS pcl_align RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(WIN32)
	add_custom_command(TARGET pcl_renderer POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different ${REALSENSE2_DLL} $<TARGET_FILE_DIR:pcl_renderer>)

	add_custom_command(TARGET pcl_align POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different ${REALSENSE2_DLL} $<TARGET_FILE_DIR:pcl_align>)

add_custom_command(TARGET pcl_align POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different "C:/Program Files/OpenNI2/Redist/OpenNI2.dll" $<TARGET_FILE_DIR:pcl_align>)
endif()
