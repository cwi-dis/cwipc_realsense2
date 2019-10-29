
cmake_minimum_required(VERSION 3.10.0)

add_executable(cwipc_multigrab cwipc_multigrab.cpp)
include_directories(
	"../../include"
	${PCL_INCLUDE_DIRS}
)
target_link_libraries(cwipc_multigrab cwipc_realsense2 cwipc_util ${PCL_LIBRARIES})

install(TARGETS cwipc_multigrab RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_test(NAME cwipc_multigrab COMMAND cwipc_multigrab 10 - )
