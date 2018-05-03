#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(MultiRealSenseCapture)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

if(BUILD_GRAPHICAL_EXAMPLES)
    add_executable(multi_camera_capture multi_camera_capture.cpp example.hpp multiFrame.cpp multiFrame.hpp)
    target_link_libraries(multi_camera_capture ${DEPENDENCIES})
    set_target_properties (multi_camera_capture PROPERTIES
        FOLDER Examples
    )

    install(
        TARGETS

        multi_camera_capture

        RUNTIME DESTINATION
        ${CMAKE_INSTALL_PREFIX}/bin
    )
endif()
