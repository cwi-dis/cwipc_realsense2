#ifndef cwipc_realsense_MFOfflineCamera_hpp
#define cwipc_realsense_MFOfflineCamera_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>

#include "defs.h"
#include "cwipc_realsense2/MFCamera.hpp"

class MFOfflineCamera : public MFCamera {
private:
	MFOfflineCamera(const MFOfflineCamera&);	// Disable copy constructor
	MFOfflineCamera& operator=(const MFOfflineCamera&);	// Disable assignment
public:
	MFOfflineCamera(rs2::context& ctx, MFCaptureConfig& configuration, int _camera_index, MFCameraData& _camData, MFOfflineSettings& settings);
	~MFOfflineCamera();

	void _start_capture_thread();
	void _capture_thread_main();
	bool feed_image_data(int frameNum, void *colorBuffer, size_t colorSize, void *depthBuffer, size_t depthSize);
private:
	int depth_width, depth_height, depth_bpp, depth_fps;
	rs2_format depth_format;
	int color_width, color_height, color_bpp, color_fps;
	rs2_format color_format;
	rs2_extrinsics depth_to_color_extrinsics;
	rs2::frameset current_frameset;
	rs2::software_device dev;
	rs2::software_sensor depth_sensor;
	rs2::software_sensor color_sensor;
	rs2::stream_profile color_stream;
	rs2::stream_profile depth_stream;
	rs2::syncer sync;
	int feedFrameNum;
};
#endif // cwipc_realsense_MFOfflineCamera_hpp
