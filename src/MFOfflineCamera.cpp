//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//

// Define to get (a little) debug prints
#undef CWIPC_DEBUG
#define CWIPC_DEBUG_THREAD

// This is the dll source, so define external symbols as dllexport on windows.

#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif

#define XXXJACK_DEPTH_W 640
#define XXXJACK_DEPTH_H 480
#define XXXJACK_DEPTH_FPS 30
#define XXXJACK_DEPTH_BPP 2
#define XXXJACK_DEPTH_FMT RS2_FORMAT_Z16
#define XXXJACK_COLOR_W 640
#define XXXJACK_COLOR_H 480
#define XXXJACK_COLOR_FPS 30
#define XXXJACK_COLOR_BPP 4
#define XXXJACK_COLOR_FMT RS2_FORMAT_RGBA8
#define XXXJACK_INTER_STREAM_EXTRINSICS { { 1,0,0,0,1,0,0,0,1 },{ 0,0,0 } }

#include "cwipc_realsense2/defs.h"
#include "cwipc_realsense2/utils.h"
#include "cwipc_realsense2/MFOfflineCamera.hpp"

MFOfflineCamera::MFOfflineCamera(rs2::context& ctx, MFCaptureConfig& configuration, int _camera_index, MFCameraData& _camData)
:	MFCamera(_camera_index, ctx, configuration, _camData),
	feed_number(0),
	dev(),
	depth_sensor(dev.add_sensor("Depth")),
	color_sensor(dev.add_sensor("Color"))
{
	dev = rs2::software_device();
	//xxxjack dev.add_to(ctx);

	// Create depth stream
	rs2_intrinsics depth_intrinsics = {
		XXXJACK_DEPTH_W, XXXJACK_DEPTH_H,
		(float)XXXJACK_DEPTH_W / 2, (float)XXXJACK_DEPTH_H / 2,
		(float)XXXJACK_DEPTH_W , (float)XXXJACK_DEPTH_H ,
		RS2_DISTORTION_BROWN_CONRADY,
		{ 0,0,0,0,0 }
	};
	depth_stream = depth_sensor.add_video_stream({
		RS2_STREAM_DEPTH,
		0,
		0,
		XXXJACK_DEPTH_W, XXXJACK_DEPTH_H,
		XXXJACK_DEPTH_FPS, XXXJACK_DEPTH_BPP,
		XXXJACK_DEPTH_FMT,
		depth_intrinsics
	});
	depth_sensor.add_read_only_option(RS2_OPTION_DEPTH_UNITS, 0.001f);

	// Create color stream
	rs2_intrinsics color_intrinsics = {
		XXXJACK_COLOR_W, XXXJACK_COLOR_H,
		(float)XXXJACK_COLOR_W / 2, (float)XXXJACK_COLOR_H / 2,
		(float)XXXJACK_COLOR_W / 2, (float)XXXJACK_COLOR_H / 2,
		RS2_DISTORTION_BROWN_CONRADY,
		{ 0,0,0,0,0 }
	};
	color_stream = color_sensor.add_video_stream({
		RS2_STREAM_COLOR,
		0,
		1,
		XXXJACK_COLOR_W, XXXJACK_COLOR_H,
		XXXJACK_COLOR_FPS, XXXJACK_COLOR_BPP,
		XXXJACK_COLOR_FMT,
		color_intrinsics
	});

	// Tie the two streams together
	dev.create_matcher(RS2_MATCHER_DLR_C);
	depth_sensor.open(depth_stream);
	color_sensor.open(color_stream);
	depth_sensor.start(sync);
	color_sensor.start(sync);
	depth_stream.register_extrinsics_to(color_stream, XXXJACK_INTER_STREAM_EXTRINSICS);
}

MFOfflineCamera::~MFOfflineCamera()
{
}

void MFOfflineCamera::_capture_thread_main()
{
	while(!stopped) {
		// Wait to find if there is a next set of frames from the camera
		rs2::frameset frames = sync.wait_for_frames();
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "MFOfflineCamera: _capture_thread_main: got " << frames.size() << " frames" << std::endl;
#endif
		captured_frame_queue.enqueue(frames);
		std::this_thread::yield();
	}
}
bool MFOfflineCamera::feed_image_data(int sensorNum, void *buffer, size_t size)
{
	if (sensorNum < 0 || sensorNum > 1) return false;
	if (sensorNum == 0) {
		int stride = XXXJACK_DEPTH_W * XXXJACK_DEPTH_BPP;
		int bpp = XXXJACK_DEPTH_BPP;
		depth_sensor.on_video_frame({
			buffer,
			[](void *) {},
			stride, bpp,
			(rs2_time_t)((feed_number/2) * 16),
			RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK,
			(feed_number/2),
			depth_stream
		});
	} else {
		int stride = XXXJACK_COLOR_W * XXXJACK_COLOR_BPP;
		int bpp = XXXJACK_COLOR_BPP;
		color_sensor.on_video_frame({
			buffer,
			[](void *) {},
			stride, bpp,
			(rs2_time_t)((feed_number/2) * 16),
			RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK,
			(feed_number/2),
			color_stream
		});
	}
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "MFOfflineCamera: fed camera " << serial << " sensor " << sensorNum << " framenum " << (feed_number/2) << std::endl;
#endif
	feed_number++;

	return true;
}
