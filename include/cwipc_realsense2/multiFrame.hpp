//
//  multiFrame.hpp
//
//  Created by Fons Kuijk on 23-04-18
//

#ifndef cwipw_realsense_multiFrame_hpp
#define cwipw_realsense_multiFrame_hpp
#pragma once

#if 0
#include <atomic>
#include <stdio.h>
#include <vector>
#include <map>
#include <iostream>
#include <functional>
#include <fstream>
#include <ctime>
#include <chrono>
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <librealsense2/rs.hpp>
#include <Eigen/StdVector>

#include "defs.h"

#ifndef CWIPC_DLL_ENTRY
#if defined(WIN32) || defined(_WIN32)
#define CWIPC_DLL_ENTRY __declspec(dllimport)
#else
#define CWIPC_DLL_ENTRY 
#endif
#endif

using namespace std::chrono;
#endif
#include "cwipc_realsense2/MFCapture.hpp"
#include "cwipc_realsense2/MFCamera.hpp"
#include "cwipc_realsense2/MFOffline.hpp"
#include "cwipc_realsense2/MFOfflineCamera.hpp"

bool MFCapture_versionCheck(char **errorMessage);
#endif /* cwipw_realsense_multiFrame_hpp */
