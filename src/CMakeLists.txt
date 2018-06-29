# ======================================================
# Compiled by Fons @ CWI, Amsterdam for VRTogether
#
# Copyright (C) 2018 by CWI. All rights reserved.
# ======================================================
#
#  minimum required cmake version: 3.12.0
cmake_minimum_required(VERSION 3.12.0)

add_executable(multi_camera_capture multi_camera_capture.cpp)
target_sources(multi_camera_capture PUBLIC multiFrame.cpp multiFrame.hpp)
target_link_libraries(multi_camera_capture ${REALSENSE2_LIB})
set_target_properties(multi_camera_capture PROPERTIES
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
#get_target_property(PROP_RESULT multi_camera_capture RUNTIME_OUTPUT_DIRECTORY_DEBUG)
#	$<IF$<CONFIG:Debug>,RUNTIME_OUTPUT_DIRECTORY_DEBUG,
#	$<IF$<CONFIG:Release>,RUNTIME_OUTPUT_DIRECTORY_RELEASE,
#	$<IF$<CONFIG:MinSizeRel>,RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL,
#	$<IF$<CONFIG:RelWithDebInfo>,RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO,>>>>

message("exec in: $<TARGET_FILE_DIR:multi_camera_capture>")
install(TARGETS multi_camera_capture RUNTIME DESTINATION D:/${PROJECT_NAME}/bin)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX D:/${PROJECT_NAME} CACHE PATH "..." FORCE)
endif()

if(BUILD_GRAPHICAL_EXAMPLES)
    target_sources(multi_camera_capture PUBLIC example.hpp )
    target_link_libraries(multi_camera_capture ${DEPENDENCIES})
endif()

add_custom_command(TARGET multi_camera_capture POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different ${REALSENSE2_DLL} $<TARGET_FILE_DIR:multi_camera_capture>)