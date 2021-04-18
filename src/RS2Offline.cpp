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
#define CWIPC_DLL_ENTRY __declspec(dllexport)
#endif

#include "cwipc_realsense2/private/defs.h"
#include "cwipc_realsense2/private/utils.h"
#include "cwipc_realsense2/private/RS2Offline.hpp"
#include "cwipc_realsense2/private/RS2OfflineCamera.hpp"

RS2Offline::RS2Offline(cwipc_rs2offline_settings& settings, const char *configFilename)
:	RS2Capture(1)
{
	bool ok = cwipc_rs2_file2config(configFilename, &configuration);
	assert(ok);
	int camera_index = 0;
	for (RS2CameraData& cd : configuration.cameraData) {
		cd.cloud = new_cwipc_pcl_pointcloud();
		auto cam = new RS2OfflineCamera(ctx, configuration, camera_index, cd, settings);
		feeders.push_back(cam);
		cameras.push_back(cam);
		camera_index++;
	}
	for (auto cam: cameras)
		cam->start_capturer();
	stopped = false;
	control_thread = new std::thread(&RS2Offline::_control_thread_main, this);
	_cwipc_setThreadName(control_thread, L"cwipc_realsense2::RS2Offline::control_thread");
}

RS2Offline::~RS2Offline() {

}

bool RS2Offline::feed_image_data(int camNum, int frameNum, void *colorBuffer, size_t colorSize,  void *depthBuffer, size_t depthSize)
{
	if (camNum < 0 || camNum > feeders.size()) return false;
	auto cam = feeders[camNum];
	return cam->feed_image_data(frameNum, colorBuffer, colorSize, depthBuffer, depthSize);
}
