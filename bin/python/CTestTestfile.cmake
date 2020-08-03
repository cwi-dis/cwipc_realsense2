# CMake generated Testfile for 
# Source directory: C:/dev2/pilot2/cwipc_realsense2/python
# Build directory: C:/dev2/pilot2/cwipc_realsense2/bin/python
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if("${CTEST_CONFIGURATION_TYPE}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test(run_python_tests "C:/ProgramData/Anaconda3/python.exe" "test_cwipc_realsense2.py")
  set_tests_properties(run_python_tests PROPERTIES  ENVIRONMENT "CWIPC_TEST_DLL=C:/dev2/pilot2/cwipc_realsense2/bin/bin//Release/cwipc_realsense2.dll;PYTHONPATH=C:/dev2/pilot2/install/share/cwipc_util/python" WORKING_DIRECTORY "C:/dev2/pilot2/cwipc_realsense2/python")
elseif("${CTEST_CONFIGURATION_TYPE}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test(run_python_tests "C:/ProgramData/Anaconda3/python.exe" "test_cwipc_realsense2.py")
  set_tests_properties(run_python_tests PROPERTIES  ENVIRONMENT "CWIPC_TEST_DLL=C:/dev2/pilot2/cwipc_realsense2/bin/bin//Release/cwipc_realsense2.dll;PYTHONPATH=C:/dev2/pilot2/install/share/cwipc_util/python" WORKING_DIRECTORY "C:/dev2/pilot2/cwipc_realsense2/python")
elseif("${CTEST_CONFIGURATION_TYPE}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
  add_test(run_python_tests "C:/ProgramData/Anaconda3/python.exe" "test_cwipc_realsense2.py")
  set_tests_properties(run_python_tests PROPERTIES  ENVIRONMENT "CWIPC_TEST_DLL=C:/dev2/pilot2/cwipc_realsense2/bin/bin//Release/cwipc_realsense2.dll;PYTHONPATH=C:/dev2/pilot2/install/share/cwipc_util/python" WORKING_DIRECTORY "C:/dev2/pilot2/cwipc_realsense2/python")
elseif("${CTEST_CONFIGURATION_TYPE}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test(run_python_tests "C:/ProgramData/Anaconda3/python.exe" "test_cwipc_realsense2.py")
  set_tests_properties(run_python_tests PROPERTIES  ENVIRONMENT "CWIPC_TEST_DLL=C:/dev2/pilot2/cwipc_realsense2/bin/bin//Release/cwipc_realsense2.dll;PYTHONPATH=C:/dev2/pilot2/install/share/cwipc_util/python" WORKING_DIRECTORY "C:/dev2/pilot2/cwipc_realsense2/python")
else()
  add_test(run_python_tests NOT_AVAILABLE)
endif()
