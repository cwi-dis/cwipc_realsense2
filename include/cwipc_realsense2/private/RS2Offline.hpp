#ifndef cwipc_realsense_RS2Offline_hpp
#define cwipc_realsense_RS2Offline_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

#include <librealsense2/rs.hpp>

#include "cwipc_realsense2/api.h"
#include "cwipc_realsense2/private/RS2Config.hpp"
#include "cwipc_realsense2/private/RS2Capture.hpp"
#include "cwipc_realsense2/private/RS2OfflineCamera.hpp"

class RS2Offline : public RS2Capture {

public:
	// methods
	RS2Offline(cwipc_rs2offline_settings& _settings);
	bool feed_image_data(int camNum, int frameNum, void *colorBuffer, size_t colorSize,  void *depthBuffer, size_t depthSize);
	~RS2Offline();
	virtual bool config_reload(const char* configFilename) override;
private:
	cwipc_rs2offline_settings settings;
	std::vector<RS2OfflineCamera*> feeders;
};
#endif // cwipc_realsense_RS2Offline_hpp
