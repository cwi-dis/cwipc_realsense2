# CMake generated Testfile for 
# Source directory: C:/dev2/pilot2/cwipc_realsense2/apps/cwipc_multigrab
# Build directory: C:/dev2/pilot2/cwipc_realsense2/bin/apps/cwipc_multigrab
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if("${CTEST_CONFIGURATION_TYPE}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test(cwipc_multigrab "C:/dev2/pilot2/cwipc_realsense2/bin/bin/Debug/cwipc_multigrab.exe" "10" "-")
elseif("${CTEST_CONFIGURATION_TYPE}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test(cwipc_multigrab "C:/dev2/pilot2/cwipc_realsense2/bin/bin/Release/cwipc_multigrab.exe" "10" "-")
elseif("${CTEST_CONFIGURATION_TYPE}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
  add_test(cwipc_multigrab "C:/dev2/pilot2/cwipc_realsense2/bin/bin/MinSizeRel/cwipc_multigrab.exe" "10" "-")
elseif("${CTEST_CONFIGURATION_TYPE}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test(cwipc_multigrab "C:/dev2/pilot2/cwipc_realsense2/bin/bin/RelWithDebInfo/cwipc_multigrab.exe" "10" "-")
else()
  add_test(cwipc_multigrab NOT_AVAILABLE)
endif()
