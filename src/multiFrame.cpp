//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//

#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif

#include "cwipc_realsense2/multiFrame.hpp"

bool MFCapture_versionCheck(char **errorMessage)
{
	int version = rs2_get_api_version(nullptr);
	if ((version/100) == (RS2_API_VERSION/100)) return true;
	if (errorMessage) {
		static char errorBuf[80];
		sprintf(errorBuf, "Built against librealsense %d but %d is installed.", RS2_API_VERSION, version);
		*errorMessage = errorBuf;
	}
	return false;
}
