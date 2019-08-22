
cmake_minimum_required(VERSION 3.10.0)

add_executable(cwipc_rs2offline cwipc_rs2offline.cpp)
include_directories(
	"../../include"
	${PCL_INCLUDE_DIRS}
)
target_link_libraries(cwipc_rs2offline cwipc_realsense2 cwipc_util ${PCL_LIBRARIES})

install(TARGETS cwipc_rs2offline RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_test(NAME cwipc_rs2offline COMMAND cwipc_rs2offline 6 . )
