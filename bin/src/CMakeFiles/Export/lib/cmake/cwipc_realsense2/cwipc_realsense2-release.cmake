#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "cwipc_realsense2" for configuration "Release"
set_property(TARGET cwipc_realsense2 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(cwipc_realsense2 PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/cwipc_realsense2.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/cwipc_realsense2.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS cwipc_realsense2 )
list(APPEND _IMPORT_CHECK_FILES_FOR_cwipc_realsense2 "${_IMPORT_PREFIX}/lib/cwipc_realsense2.lib" "${_IMPORT_PREFIX}/bin/cwipc_realsense2.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
