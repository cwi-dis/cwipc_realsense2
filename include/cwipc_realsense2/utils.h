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

#include "cwipc_realsense2/multiFrame.hpp"

_CWIPC_REALSENSE2_EXPORT bool file2config(const char* filename, configdata* config);

// store the current camera transformation setting into a xml document
_CWIPC_REALSENSE2_EXPORT void config2file(char* filename, configdata* config);

typedef struct HsvColor
{
	unsigned char h;
	unsigned char s;
	unsigned char v;
} HsvColor;

_CWIPC_REALSENSE2_EXPORT cwipc_pcl_point* hsvToRgb(HsvColor hsv, cwipc_pcl_point* pnt);

_CWIPC_REALSENSE2_EXPORT HsvColor rgbToHsv(cwipc_pcl_point* pnt);

_CWIPC_REALSENSE2_EXPORT bool noChromaRemoval(cwipc_pcl_point* p);

#endif /* cwipw_realsense_utils_h */
