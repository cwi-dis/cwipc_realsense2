
cmake_minimum_required(VERSION 3.16.0)

add_executable(cwipc_rs2offline cwipc_rs2offline.cpp stb_image.h)
include_directories(
	"../../include"
	${PCL_INCLUDE_DIRS}
)
target_link_libraries(cwipc_rs2offline cwipc_realsense2 cwipc_util)

install(TARGETS cwipc_rs2offline RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_test(NAME cwipc_rs2offline COMMAND cwipc_rs2offline input/offlineconfig.xml input/color.png input/depth.png output/rs2offline.ply WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../tests/fixtures )
