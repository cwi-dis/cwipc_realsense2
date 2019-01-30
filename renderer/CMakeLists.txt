# ======================================================
# Compiled by Fons @ CWI, Amsterdam for VRTogether
#
# Copyright (C) 2018 by CWI. All rights reserved.
# ======================================================
#
#  minimum required cmake version: 3.12.0
cmake_minimum_required(VERSION 3.12.0)

add_executable(pcl_renderer pcl_renderer.cpp pcl_renderer.hpp)
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

aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/../src MULTIFRAME_SRC)
include_directories(../src)

target_sources(pcl_align PUBLIC ${MULTIFRAME_SRC} ${TINY_SRC} )

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

add_custom_command(TARGET pcl_renderer POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different ${REALSENSE2_DLL} $<TARGET_FILE_DIR:pcl_renderer>)

add_custom_command(TARGET pcl_align POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different ${REALSENSE2_DLL} $<TARGET_FILE_DIR:pcl_align>)

add_custom_command(TARGET pcl_align POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different "C:/Program Files/OpenNI2/Redist/OpenNI2.dll" $<TARGET_FILE_DIR:pcl_align>)
