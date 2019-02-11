

set(CMAKECONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/cwipc_realsense2")

# add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
# 
# include(CMakePackageConfigHelpers)
# 
# write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/realsense2ConfigVersion.cmake"
#     VERSION ${REALSENSE_VERSION_STRING} COMPATIBILITY AnyNewerVersion)
# 
# configure_package_config_file(CMake/realsense2Config.cmake.in realsense2Config.cmake
#     INSTALL_DESTINATION ${CMAKECONFIG_INSTALL_DIR}
#     INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/bin
#     PATH_VARS CMAKE_INSTALL_INCLUDEDIR
# )
# 
# configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake" "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" IMMEDIATE @ONLY)
# configure_file(config/librealsense.pc.in config/realsense2.pc @ONLY)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/cwipc_realsense
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

#install(EXPORT cwipcrsTargets
#        FILE cwipcrsTargets.cmake
#        NAMESPACE cwipc_realsense2::
#        DESTINATION ${CMAKECONFIG_INSTALL_DIR}
#)

# install(FILES "${CMAKE_CURRENT_BINARY_DIR}/realsense2Config.cmake"
#         DESTINATION ${CMAKECONFIG_INSTALL_DIR}
# )
# 
# install(FILES "${CMAKE_CURRENT_BINARY_DIR}/realsense2ConfigVersion.cmake"
#         DESTINATION ${CMAKECONFIG_INSTALL_DIR}
# )

install(CODE "execute_process(COMMAND ldconfig)")

# Set library pkgconfig file for facilitating 3rd party integration
# install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config/cwipc_realsense.pc"
#         DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig"
# )
