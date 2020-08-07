
cmake_minimum_required(VERSION 3.10.0)

add_executable(cwipc_grab cwipc_grab.cpp)
include_directories(
	"../../include"
	${PCL_INCLUDE_DIRS}
)
target_link_libraries(cwipc_grab cwipc_realsense2 cwipc_util ${PCL_LIBRARIES})

install(TARGETS cwipc_grab RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_test(NAME cwipc_grab COMMAND cwipc_grab 2 . )
set_tests_properties(cwipc_grab PROPERTIES ENVIRONMENT CWIPC_REALSENSE2_TESTING=yes)
