//
//  utils.h
//
//  Created by Fons Kuijk on 12-12-18.
//

#ifndef cwipw_realsense_utils_h
#define cwipw_realsense_utils_h
#pragma once

#ifndef _CWIPC_REALSENSE2_EXPORT
#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllimport)
#else
#define _CWIPC_REALSENSE2_EXPORT 
#endif
#endif

#include <cstdint>
#include <thread>
#include "cwipc_util/api_pcl.h"

struct RS2CaptureConfig;

_CWIPC_REALSENSE2_EXPORT void cwipc_rs2_log_warning(std::string warning);
_CWIPC_REALSENSE2_EXPORT extern char **cwipc_rs2_warning_store;

_CWIPC_REALSENSE2_EXPORT bool cwipc_rs2_file2config(const char* filename, RS2CaptureConfig* config);

// store the current camera transformation setting into a xml document
_CWIPC_REALSENSE2_EXPORT void cwipc_rs2_config2file(const char* filename, RS2CaptureConfig* config);

_CWIPC_REALSENSE2_EXPORT bool cwipc_rs2_noChromaRemoval(cwipc_pcl_point* p);

#ifdef _WIN32
#include <Windows.h>
inline void _cwipc_setThreadName(std::thread* thr, const wchar_t* name) {
	HANDLE threadHandle = static_cast<HANDLE>(thr->native_handle());
	SetThreadDescription(threadHandle, name);
}
#else
inline void _cwipc_setThreadName(std::thread* thr, const wchar_t* name) {}
#endif
#endif /* cwipw_realsense_utils_h */
