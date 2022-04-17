
cmake_minimum_required(VERSION 3.16.0)

add_executable(cwipc_rs2multigrab cwipc_rs2multigrab.cpp)
include_directories(
	"../../include"
	${PCL_INCLUDE_DIRS}
)
target_link_libraries(cwipc_rs2multigrab cwipc_realsense2 cwipc_util)

# install(TARGETS cwipc_rs2multigrab RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_test(NAME cwipc_rs2multigrab COMMAND cwipc_rs2multigrab 10 - )
set_tests_properties(cwipc_rs2multigrab PROPERTIES ENVIRONMENT CWIPC_REALSENSE2_TESTING=yes)
set_tests_properties(cwipc_rs2multigrab PROPERTIES RUN_SERIAL TRUE)
