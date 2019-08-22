//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//

// Define to get (a little) debug prints
#undef CWIPC_DEBUG
#undef CWIPC_DEBUG_THREAD

// This is the dll source, so define external symbols as dllexport on windows.

#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif

#include "cwipc_realsense2/defs.h"
#include "cwipc_realsense2/utils.h"
#include "cwipc_realsense2/MFOfflineCamera.hpp"

MFOfflineCamera::MFOfflineCamera(rs2::context& ctx, MFCaptureConfig& configuration, int _camera_index, MFCameraData& _camData)
:	MFCamera(ctx, configuration, _camera_index, _camData, "2")
{
}

MFOfflineCamera::~MFOfflineCamera()
{
}

void MFOfflineCamera::start_capturer()
{
	MFCamera::start_capturer();
}

