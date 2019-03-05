# ======================================================
# Compiled by Fons @ CWI, Amsterdam for VRTogether
#
# Copyright (C) 2018 by CWI. All rights reserved.
# ======================================================
#
#  minimum required cmake version: 3.10.0
cmake_minimum_required(VERSION 3.10.0)


add_library(cwipc_realsense2 SHARED cwipc_realsense2.cpp multiFrame.cpp utils.cpp ${TINY_SRC})

target_sources(cwipc_realsense2
	PRIVATE
	"../include/cwipc_realsense2/defs.h"
	"../include/cwipc_realsense2/utils.h"
	"../include/cwipc_realsense2/multiFrame.hpp"
	"../include/cwipc_realsense2/window_util.hpp"
	"../include/cwipc_realsense2/api.h"
)

target_link_libraries(cwipc_realsense2 cwipc_util realsense2 ${PCL_LIBRARIES})

install(TARGETS cwipc_realsense2
    EXPORT cwipc_realsense2
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_PREFIX}/include/cwipc_realsense2"
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/cwipc_realsense2
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${PROJECT_SOURCE_DIR}/CMakeFiles/cwipc_realsense2-config.cmake DESTINATION lib/cmake/cwipc_realsense2)

install(EXPORT cwipc_realsense2 DESTINATION lib/cmake/cwipc_realsense2)

# Set library pkgconfig file for facilitating 3rd party integration
# install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config/cwipc_realsense.pc"
#         DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig"
# )
