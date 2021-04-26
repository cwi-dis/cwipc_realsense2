
cmake_minimum_required(VERSION 3.10.0)

add_executable(cwipc_rs2grab cwipc_rs2grab.cpp)
include_directories(
	"../../include"
	${PCL_INCLUDE_DIRS}
)
target_link_libraries(cwipc_rs2grab cwipc_realsense2 cwipc_util ${PCL_LIBRARIES})

# install(TARGETS cwipc_rs2grab RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_test(NAME cwipc_rs2grab COMMAND cwipc_rs2grab 2 . )
set_tests_properties(cwipc_rs2grab PROPERTIES ENVIRONMENT CWIPC_REALSENSE2_TESTING=yes)
