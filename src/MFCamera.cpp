//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//

// Define to get (a little) debug prints
#define CWIPC_DEBUG
#define CWIPC_DEBUG_THREAD

// This is the dll source, so define external symbols as dllexport on windows.

#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif

#if 0
#include <chrono>
#include <cstdint>
#include "cwipc_realsense2/multiFrame.hpp"
#endif

#include "cwipc_realsense2/defs.h"
#include "cwipc_realsense2/utils.h"
#include "cwipc_realsense2/MFCamera.hpp"

#ifdef WITH_DUMP_VIDEO_FRAMES
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "cwipc_realsense2/stb_image_write.h"
#endif

// Internal-only constructor for OfflineCamera constructor
MFCamera::MFCamera(int _camera_index, rs2::context& ctx, MFCaptureConfig& configuration, MFCameraData& _camData)
:	minx(0), minz(0), maxz(0),
	camera_index(_camera_index),
	serial(_camData.serial),
	stopped(true),
	captured_frame_queue(1),
	camData(_camData),
	camSettings(configuration.default_camera_settings),
	high_speed_connection(true),
	camera_width(0),
	camera_height(0),
	camera_fps(0),
	do_depth_filtering(configuration.depth_filtering),
	do_background_removal(configuration.background_removal),
	do_greenscreen_removal(configuration.greenscreen_removal),
	grabber_thread(nullptr),
	processing_frame_queue(1),
	pipe(ctx),
	aligner(RS2_STREAM_DEPTH)
{
}

MFCamera::MFCamera(rs2::context& ctx, MFCaptureConfig& configuration, int _camera_index, MFCameraData& _camData, std::string _usb)
:	minx(0), minz(0), maxz(0),
	camera_index(_camera_index),
	serial(_camData.serial),
	stopped(true),
	captured_frame_queue(1),
	camData(_camData),
	camSettings(configuration.default_camera_settings),
	high_speed_connection(_usb[0] == '3'),
	camera_width(high_speed_connection ? configuration.usb3_width : configuration.usb2_width),
	camera_height(high_speed_connection ? configuration.usb3_height : configuration.usb2_height),
	camera_fps(high_speed_connection ? configuration.usb3_fps : configuration.usb2_fps),
	do_depth_filtering(configuration.depth_filtering),
	do_background_removal(configuration.background_removal),
	do_greenscreen_removal(configuration.greenscreen_removal),
	grabber_thread(nullptr),
	processing_frame_queue(1),
	pipe(ctx),
	aligner(RS2_STREAM_DEPTH)
{
#ifdef CWIPC_DEBUG
		std::cout << "MFCapture: creating camera " << serial << std::endl;
#endif
	_init_filters();
}

MFCamera::~MFCamera()
{
#ifdef CWIPC_DEBUG
	std::cout << "MFCamera: destroying " << serial << std::endl;
#endif
	assert(stopped);
}

void MFCamera::_init_filters()
{
	if (!do_depth_filtering) return;
	dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, camSettings.decimation_value);

	threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, camSettings.threshold_near);
	threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, camSettings.threshold_far);

	spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, camSettings.spatial_iterations);
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, camSettings.spatial_alpha);
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, camSettings.spatial_delta);
	spat_filter.set_option(RS2_OPTION_HOLES_FILL, camSettings.spatial_filling);

	temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, camSettings.temporal_alpha);
	temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, camSettings.temporal_delta);
	temp_filter.set_option(RS2_OPTION_HOLES_FILL, camSettings.temporal_percistency);
}

void MFCamera::capture_frameset()
{
	current_frameset = captured_frame_queue.wait_for_frame();
}

// Configure and initialize caputuring of one camera
void MFCamera::start()
{
	assert(stopped);
	rs2::config cfg;
	std::cerr << "cwipc_realsense2: multiFrame: starting camera " << serial << ": " << camera_width << "x" << camera_height << "@" << camera_fps << std::endl;
	cfg.enable_device(serial);
	cfg.enable_stream(RS2_STREAM_COLOR, camera_width, camera_height, RS2_FORMAT_RGB8, camera_fps);
	cfg.enable_stream(RS2_STREAM_DEPTH, camera_width, camera_height, RS2_FORMAT_Z16, camera_fps);
	// xxxjack need to set things like disabling color correction and auto-exposure
	// xxxjack need to allow setting things like laser power
	pipe.start(cfg);		// Start streaming with the configuration just set
}

void MFCamera::stop()
{
	assert(!stopped);
	assert(grabber_thread);
	stopped = true;
	if (grabber_thread) grabber_thread->join();
	if (processing_thread) processing_thread->join();
	pipe.stop();
}

void MFCamera::start_capturer()
{
	assert(stopped);
	stopped = false;
	grabber_thread = new std::thread(&MFCamera::_capture_thread_main, this);
	processing_thread = new std::thread(&MFCamera::_processing_thread_main, this);
}

void MFCamera::_capture_thread_main()
{
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "frame capture: cam=" << serial << " thread started" << std::endl;
#endif
	while(!stopped) {
		// Wait to find if there is a next set of frames from the camera
		rs2::frameset frames = pipe.wait_for_frames();
#ifdef CWIPC_DEBUG_THREAD
		std::cerr << "frame capture: cam=" << serial << ", seq=" << frames.get_frame_number() << std::endl;
#endif
		captured_frame_queue.enqueue(frames);
		std::this_thread::yield();
	}
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "frame capture: cam=" << serial << " thread stopped" << std::endl;
#endif
}

void MFCamera::_processing_thread_main()
{
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "frame processing: cam=" << serial << " thread started" << std::endl;
#endif
	while(!stopped) {
		// Wait for next frame to process. Allow aborting in case of stopped becoming false...
		rs2::frameset processing_frameset;
		bool ok = processing_frame_queue.try_wait_for_frame(&processing_frameset, 1000);
		if (!ok) continue;

		std::lock_guard<std::mutex> lock(processing_mutex);

		if (do_depth_filtering) {
			processing_frameset = processing_frameset.apply_filter(aligner);
			processing_frameset = processing_frameset.apply_filter(dec_filter);
			processing_frameset = processing_frameset.apply_filter(threshold_filter);
			processing_frameset = processing_frameset.apply_filter(depth_to_disparity);
			processing_frameset = processing_frameset.apply_filter(spat_filter);
			processing_frameset = processing_frameset.apply_filter(temp_filter);
			processing_frameset = processing_frameset.apply_filter(disparity_to_depth);
		}

		rs2::depth_frame depth = processing_frameset.get_depth_frame();
		rs2::video_frame color = processing_frameset.get_color_frame();
		assert(depth);
		assert(color);
#ifdef CWIPC_DEBUG
		std::cerr << "frame processing: cam=" << serial << ", depthseq=" << depth.get_frame_number() << ", colorseq=" << depth.get_frame_number() << std::endl;
#endif

		// Calculate new pointcloud, map to the color images and get vertices and color indices
		auto points = pointcloud.calculate(depth);
		pointcloud.map_to(color);
		auto vertices = points.get_vertices();
		auto texture_coordinates = points.get_texture_coordinates();

		// Get some constants used later to map colors and such from rs2 to pcl pointclouds.
		const int texture_width = color.get_width();
		const int texture_height = color.get_height();
		const int texture_x_step = color.get_bytes_per_pixel();
		const int texture_y_step = color.get_stride_in_bytes();
		const unsigned char *texture_data = (unsigned char*)color.get_data();
		const uint8_t camera_label = (uint8_t)1 << camera_index;

		// Clear the previous pointcloud and pre-allocate space in the pointcloud (so we don't realloc)
		camData.cloud->clear();
		camData.cloud->reserve(points.size());

#ifdef WITH_MANUAL_BACKGROUND_REMOVAL
		// Note by Jack: this code is currently not correct, hasn't been updated
		// for the texture coordinate mapping.
		if (do_background_removal) {

			// Set the background removal window
			if (camData.background.z > 0.0) {
				maxz = camData.background.z;
				minz = 0.0;
				if (camData.background.x != 0.0) {
					minx = camData.background.x;
				}
				else {
					for (int i = 0; i < points.size(); i++) {
						double minz = 100;
						if (vertices[i].z != 0 && minz > vertices[i].z) {
							minz = vertices[i].z;
							minx = vertices[i].x;
						}
					}
				}
			}
			else {
				minz = 100.0;
				for (int i = 0; i < points.size(); i++) {
					if (vertices[i].z != 0 && minz > vertices[i].z) {
						minz = vertices[i].z;
						minx = vertices[i].x;
					}
				}
				maxz = 0.8f + minz;
			}
			// Make PointCloud
			for (int i = 0; i < points.size(); i++) {
				double x = minx - vertices[i].x; x *= x;
				double z = vertices[i].z;
				if (minz < z && z < maxz - x) { // Simple background removal, horizontally parabolic, vertically straight.
					cwipc_pcl_point pt;
					pt.x = vertices[i].x;
					pt.y = -vertices[i].y;
					pt.z = -z;
					int pi = i * 3;
					pt.r = texture_data[pi];
					pt.g = texture_data[pi + 1];
					pt.b = texture_data[pi + 2];
					pt.a = camera_label;
					if (!do_greenscreen_removal || mf_noChromaRemoval(&pt)) // chromakey removal
						camData.cloud->push_back(pt);
				}
			}
		}
		else
#endif // WITH_MANUAL_BACKGROUND_REMOVAL
		{
			// Make PointCloud
			for (int i = 0; i < points.size(); i++) {
				// Skip points with z=0 (they don't exist)
				if (vertices[i].z == 0) continue;

				cwipc_pcl_point pt;

				pt.x = vertices[i].x;
				pt.y = -vertices[i].y;
				pt.z = -vertices[i].z;

				float u = texture_coordinates[i].u;
				float v = texture_coordinates[i].v;
				int texture_x = std::min(std::max(int(u*texture_width + .5f), 0), texture_width - 1);
				int texture_y = std::min(std::max(int(v*texture_height + .5f), 0), texture_height - 1);
				int idx = texture_x * texture_x_step + texture_y * texture_y_step;
				pt.r = texture_data[idx];
				pt.g = texture_data[idx + 1];
				pt.b = texture_data[idx + 2];
				pt.a = camera_label;
				if (!do_greenscreen_removal || mf_noChromaRemoval(&pt)) // chromakey removal
					camData.cloud->push_back(pt);
			}
		}
		transformPointCloud(*camData.cloud, *camData.cloud, *camData.trafo);
		// Notify wait_for_pc that we're done.
		processing_done = true;
		processing_done_cv.notify_one();
	}
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "frame processing: cam=" << serial << " thread stopped" << std::endl;
#endif
}

void MFCamera::create_pc_from_frames()
{
	processing_frame_queue.enqueue(current_frameset);
}

void MFCamera::wait_for_pc()
{
	std::unique_lock<std::mutex> lock(processing_mutex);
	processing_done_cv.wait(lock, [this]{ return processing_done; });
	processing_done = false;
}

uint64_t MFCamera::get_capture_timestamp()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void
MFCamera::dump_color_frame(const std::string& filename)
{
#ifdef WITH_DUMP_VIDEO_FRAMES
		rs2::video_frame color = current_frameset.get_color_frame();
		stbi_write_png(filename.c_str(), color.get_width(), color.get_height(),
			color.get_bytes_per_pixel(), color.get_data(), color.get_stride_in_bytes());
#endif
}

