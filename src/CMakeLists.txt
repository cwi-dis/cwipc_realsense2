# ======================================================
# Compiled by Fons @ CWI, Amsterdam for VRTogether
#
# Copyright (C) 2018 by CWI. All rights reserved.
# ======================================================
#
cmake_minimum_required(VERSION 3.16.0)


add_library(cwipc_realsense2 
	SHARED 
	cwipc_realsense2.cpp 
	RS2Capture.cpp 
	RS2Camera.cpp 
	RS2PlaybackCapture.cpp 
	RS2PlaybackCamera.cpp 
	RS2Offline.cpp 
	RS2OfflineCamera.cpp 
	RS2Config.cpp 
	cwipc_pcl_additions.cpp 
	${TINY_SRC}
)

target_sources(cwipc_realsense2
	PRIVATE
	"../include/cwipc_realsense2/private/RS2Config.hpp"
	"../include/cwipc_realsense2/private/RS2Capture.hpp"
	"../include/cwipc_realsense2/private/RS2Camera.hpp"
	"../include/cwipc_realsense2/private/RS2PlaybackCapture.hpp"
	"../include/cwipc_realsense2/private/RS2PlaybackCamera.hpp"
	"../include/cwipc_realsense2/private/RS2Offline.hpp"
	"../include/cwipc_realsense2/private/RS2OfflineCamera.hpp"
	"../include/cwipc_realsense2/api.h"
)

target_link_libraries(cwipc_realsense2 PUBLIC cwipc_util)
target_link_libraries(cwipc_realsense2 PRIVATE ${PCL_LIBRARIES} REALSENSE2::REALSENSE2)
target_link_libraries(cwipc_realsense2 PRIVATE nlohmann_json::nlohmann_json)

install(TARGETS cwipc_realsense2
    EXPORT cwipc_realsense2
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION "include/cwipc_realsense2"
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/cwipc_realsense2
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${PROJECT_SOURCE_DIR}/CMakeFiles/cwipc_realsense2-config.cmake DESTINATION lib/cmake/cwipc_realsense2)

install(EXPORT cwipc_realsense2 DESTINATION lib/cmake/cwipc_realsense2)

if(WIN32)
	# Copy the dependent DLLs that cmake/vcpkg have created
	install(FILES $<TARGET_RUNTIME_DLLS:cwipc_realsense2> DESTINATION ${CMAKE_INSTALL_BINDIR})
	# Copy the PDB file, if it exists
	install(FILES $<TARGET_PDB_FILE:cwipc_realsense2> DESTINATION ${CMAKE_INSTALL_BINDIR} OPTIONAL)
endif()
