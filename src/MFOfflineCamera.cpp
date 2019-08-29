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

#include "cwipc_realsense2/defs.h"
#include "cwipc_realsense2/utils.h"
#include "cwipc_realsense2/MFOfflineCamera.hpp"

MFOfflineCamera::MFOfflineCamera(rs2::context& ctx, MFCaptureConfig& configuration, int _camera_index, MFCameraData& _camData)
:	MFCamera(_camera_index, ctx, configuration, _camData),
	feed_number(0),
	depth_width(640),
	depth_height(480),
	depth_bpp(2),
	depth_fps(60),
	depth_format(RS2_FORMAT_Z16),
	color_width(640),
	color_height(480),
	color_bpp(4),
	color_fps(60),
	color_format(RS2_FORMAT_RGBA8),
	depth_to_color_extrinsics({ { 1,0,0,0,1,0,0,0,1 },{ 0,0,0 } }),
	dev(),
	depth_sensor(dev.add_sensor("Depth")),
	color_sensor(dev.add_sensor("Color"))
{
	dev = rs2::software_device();
	//xxxjack dev.add_to(ctx);

	// Create depth stream
	rs2_intrinsics depth_intrinsics = {
		depth_width, depth_height,
		(float)depth_width / 2, (float)depth_height / 2,
		(float)depth_width , (float)depth_height ,
		RS2_DISTORTION_BROWN_CONRADY,
		{ 0,0,0,0,0 }
	};
	depth_stream = depth_sensor.add_video_stream({
		RS2_STREAM_DEPTH,
		0,
		0,
		depth_width, depth_height,
		depth_fps, depth_bpp,
		depth_format,
		depth_intrinsics
	});
	depth_sensor.add_read_only_option(RS2_OPTION_DEPTH_UNITS, 0.001f);

	// Create color stream
	rs2_intrinsics color_intrinsics = {
		color_width, color_height,
		(float)color_width / 2, (float)color_height / 2,
		(float)color_width / 2, (float)color_height / 2,
		RS2_DISTORTION_BROWN_CONRADY,
		{ 0,0,0,0,0 }
	};
	color_stream = color_sensor.add_video_stream({
		RS2_STREAM_COLOR,
		0,
		1,
		color_width, color_height,
		color_fps, color_bpp,
		color_format,
		color_intrinsics
	});

	// Tie the two streams together
	dev.create_matcher(RS2_MATCHER_DLR_C);
	depth_sensor.open(depth_stream);
	color_sensor.open(color_stream);
	depth_sensor.start(sync);
	color_sensor.start(sync);
	color_stream.register_extrinsics_to(depth_stream, depth_to_color_extrinsics);
}

MFOfflineCamera::~MFOfflineCamera()
{
}

void MFOfflineCamera::_capture_thread_main()
{
	while(!stopped) {
		// Wait to find if there is a next set of frames from the camera
		rs2::frameset fset;
		bool ok = sync.try_wait_for_frames(&fset);
		if (!ok) continue;
		auto depth = fset.first_or_default(RS2_STREAM_DEPTH);
		auto color = fset.first_or_default(RS2_STREAM_COLOR);
		if (!depth) {
			std::cerr << "MFOfflineCamera: warning: feed didn't produce depth frame" << std::endl;
			continue;
		}
		if (!color) {
			std::cerr << "MFOfflineCamera: warning: feed didn't produce color frame" << std::endl;
			continue;
		}
		captured_frame_queue.enqueue(fset);
		std::this_thread::yield();
	}
}

bool MFOfflineCamera::feed_image_data(void *colorBuffer, size_t colorSize,  void *depthBuffer, size_t depthSize)
{
	{
		depth_sensor.on_video_frame({
			depthBuffer,
			[](void *) {},
			depth_width * depth_bpp,
			depth_bpp,
			(rs2_time_t)(feed_number * 16),
			RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK,
			feed_number,
			depth_stream
		});
	}
	{
		color_sensor.on_video_frame({
			colorBuffer,
			[](void *) {},
			color_width*color_bpp,
			color_bpp,
			(rs2_time_t)(feed_number * 16),
			RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK,
			feed_number,
			color_stream
		});
	}
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "MFOfflineCamera: fed camera " << serial << " framenum " << feed_number << std::endl;
#endif
	feed_number++;
	return true;
}
