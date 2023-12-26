
cmake_minimum_required(VERSION 3.16.0)

add_executable(cwipc_rs2playback_grab cwipc_rs2playback_grab.cpp)
include_directories(
	"../../include"
	${PCL_INCLUDE_DIRS}
)
target_link_libraries(cwipc_rs2playback_grab cwipc_realsense2 cwipc_util)

# install(TARGETS cwipc_rs2grab RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_test(NAME cwipc_rs2playback_grab COMMAND cwipc_rs2grab cameraconfig.json 2 . )
set_tests_properties(cwipc_rs2playback_grab PROPERTIES ENVIRONMENT CWIPC_REALSENSE2_TESTING=yes)
set_tests_properties(cwipc_rs2playback_grab PROPERTIES RUN_SERIAL TRUE)
