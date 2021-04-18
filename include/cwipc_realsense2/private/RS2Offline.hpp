#ifndef cwipc_realsense_RS2Offline_hpp
#define cwipc_realsense_RS2Offline_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

#include <librealsense2/rs.hpp>

#include "cwipc_realsense2/api.h"
#include "cwipc_realsense2/private/defs.h"
#include "cwipc_realsense2/private/RS2Capture.hpp"
#include "cwipc_realsense2/private/RS2OfflineCamera.hpp"

#ifndef CWIPC_DLL_ENTRY
#if defined(WIN32) || defined(_WIN32)
#define CWIPC_DLL_ENTRY __declspec(dllimport)
#else
#define CWIPC_DLL_ENTRY 
#endif
#endif

class CWIPC_DLL_ENTRY RS2Offline : public RS2Capture {

public:
	// methods
	RS2Offline(cwipc_rs2offline_settings& settings, const char *configFilename=NULL);
	bool feed_image_data(int camNum, int frameNum, void *colorBuffer, size_t colorSize,  void *depthBuffer, size_t depthSize);
	~RS2Offline();
private:
	std::vector<RS2OfflineCamera*> feeders;
};
#endif // cwipc_realsense_RS2Offline_hpp
