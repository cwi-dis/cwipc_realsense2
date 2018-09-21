# ======================================================
# Compiled by Fons @ CWI, Amsterdam for VRTogether
#
# Copyright (C) 2018 by CWI. All rights reserved.
# ======================================================
#
#  minimum required cmake version: 3.12.0
cmake_minimum_required(VERSION 3.12.0)

add_executable(pcl_renderer pcl_renderer.cpp pcl_renderer.hpp)

target_sources(pcl_renderer PUBLIC multiFrame.cpp multiFrame.hpp PRIVATE ${TINYSRC} )

target_link_libraries(pcl_renderer ${REALSENSE2_LIB})
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
#get_target_property(PROP_RESULT pcl_renderer RUNTIME_OUTPUT_DIRECTORY_DEBUG)
#	$<IF$<CONFIG:Debug>,RUNTIME_OUTPUT_DIRECTORY_DEBUG,
#	$<IF$<CONFIG:Release>,RUNTIME_OUTPUT_DIRECTORY_RELEASE,
#	$<IF$<CONFIG:MinSizeRel>,RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL,
#	$<IF$<CONFIG:RelWithDebInfo>,RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO,>>>>

install(TARGETS pcl_renderer RUNTIME DESTINATION D:/${PROJECT_NAME}/bin)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX D:/${PROJECT_NAME} CACHE PATH "..." FORCE)
endif()

if(BUILD_GRAPHICAL_EXAMPLES)
    target_link_libraries(pcl_renderer ${DEPENDENCIES})
endif()

add_library(multiFrame SHARED multiFrame.cpp multiFrame.hpp  ${TINYSRC} )
target_link_libraries(multiFrame ${REALSENSE2_LIB})

add_custom_command(TARGET pcl_renderer POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different ${REALSENSE2_DLL} $<TARGET_FILE_DIR:pcl_renderer>)