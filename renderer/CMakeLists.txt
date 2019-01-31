# ======================================================
# Compiled by Fons @ CWI, Amsterdam for VRTogether
#
# Copyright (C) 2018 by CWI. All rights reserved.
# ======================================================
#
#  minimum required cmake version: 3.10.0
cmake_minimum_required(VERSION 3.10.0)

list(APPEND DEPENDENCIES ${LIBCWIPC_REALSENSE2_LIBRARIES})
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
	add_subdirectory(third-party/glfw)
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
    
add_executable(pcl_renderer pcl_renderer.cpp pcl_renderer.hpp ${TINY_SRC})

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

add_executable(pcl_align pcl_align.cpp pcl_align.hpp)
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

#aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/../src MULTIFRAME_SRC)

#target_sources(pcl_renderer PUBLIC ${TINY_SRC} )
#target_sources(pcl_align PUBLIC ${MULTIFRAME_SRC} ${TINY_SRC} )

target_link_libraries(pcl_renderer ${REALSENSE2_LIB})
target_link_libraries(pcl_align ${REALSENSE2_LIB})

#get_target_property(PROP_RESULT pcl_renderer RUNTIME_OUTPUT_DIRECTORY_DEBUG)
#	$<IF$<CONFIG:Debug>,RUNTIME_OUTPUT_DIRECTORY_DEBUG,
#	$<IF$<CONFIG:Release>,RUNTIME_OUTPUT_DIRECTORY_RELEASE,
#	$<IF$<CONFIG:MinSizeRel>,RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL,
#	$<IF$<CONFIG:RelWithDebInfo>,RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO,>>>>


if(BUILD_GRAPHICAL_EXAMPLES)
    target_link_libraries(pcl_renderer ${DEPENDENCIES})
    target_link_libraries(pcl_align ${DEPENDENCIES})
endif()

if(WIN32)
	add_custom_command(TARGET pcl_renderer POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different ${REALSENSE2_DLL} $<TARGET_FILE_DIR:pcl_renderer>)

	add_custom_command(TARGET pcl_align POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different ${REALSENSE2_DLL} $<TARGET_FILE_DIR:pcl_align>)
endif()
