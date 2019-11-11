//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//
#include <cstdlib>

// Define to get (a little) debug prints
#undef CWIPC_DEBUG
#undef CWIPC_DEBUG_THREAD

// This is the dll source, so define external symbols as dllexport on windows.

#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif

#include <librealsense2/rsutil.h>

#include "cwipc_realsense2/defs.h"
#include "cwipc_realsense2/utils.h"
#include "cwipc_realsense2/MFCamera.hpp"

#ifdef WITH_DUMP_VIDEO_FRAMES
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "cwipc_realsense2/stb_image_write.h"
#endif

// Internal-only constructor for OfflineCamera constructor
MFCamera::MFCamera(int _camera_index, rs2::context& ctx, MFCaptureConfig& configuration, MFCameraData& _camData)
:	pointSize(0), minx(0), minz(0), maxz(0),
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
	do_height_filtering(configuration.height_min != configuration.height_max),
	height_min(configuration.height_min),
	height_max(configuration.height_max),
	grabber_thread(nullptr),
	processing_frame_queue(1),
	pipe(ctx),
	pipe_started(false),
	aligner(RS2_STREAM_DEPTH)
{
	_init_filters();
}

MFCamera::MFCamera(rs2::context& ctx, MFCaptureConfig& configuration, int _camera_index, MFCameraData& _camData, std::string _usb)
:	pointSize(0), minx(0), minz(0), maxz(0),
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
	do_height_filtering(configuration.height_min != configuration.height_max),
	height_min(configuration.height_min),
	height_max(configuration.height_max),
	grabber_thread(nullptr),
	processing_frame_queue(1),
	pipe(ctx),
	pipe_started(false),
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
	if (camSettings.do_decimation) {
		dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, camSettings.decimation_value);
	}
	if (camSettings.do_threshold) {
		threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, camSettings.threshold_near);
		threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, camSettings.threshold_far);
	}

	if (camSettings.do_spatial) {
		spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, camSettings.spatial_iterations);
		spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, camSettings.spatial_alpha);
		spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, camSettings.spatial_delta);
		spat_filter.set_option(RS2_OPTION_HOLES_FILL, camSettings.spatial_filling);
	}

	if (camSettings.do_temporal) {
		temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, camSettings.temporal_alpha);
		temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, camSettings.temporal_delta);
		temp_filter.set_option(RS2_OPTION_HOLES_FILL, camSettings.temporal_percistency);
	}
}

bool MFCamera::capture_frameset()
{
	return captured_frame_queue.try_wait_for_frame(&current_frameset);
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
	auto profile = pipe.start(cfg);		// Start streaming with the configuration just set
	_computePointSize(profile);
	pipe_started = true;
}

void MFCamera::_computePointSize(rs2::pipeline_profile profile)
{

	// Get the 3D distance between camera and (0,0,0) or use 1m if unreasonable
	float tx = (*camData.trafo)(0,3);
	float ty = (*camData.trafo)(1,3);
	float tz = (*camData.trafo)(2,3);
	float dist = sqrt(tx*tx + ty*ty + tz*tz);
	if (dist == 0) dist = 1;

	// Now get the intrinsics for the depth stream
	auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	auto intrinsics = stream.get_intrinsics(); // Calibration data

	// Compute 2D coordinates of adjacent pixels in the middle of the field of view
	float pixel0[2], pixel1[2];
	pixel0[0] = camera_width / 2;
	pixel0[1] = camera_height / 2;
	if (do_depth_filtering && camSettings.do_decimation) {
		pixel1[0] = pixel0[0] + camSettings.decimation_value;
		pixel1[1] = pixel0[1] + camSettings.decimation_value;
	} else {
		pixel1[0] = pixel0[0] + 1;
		pixel1[1] = pixel0[1] + 1;
	}

	// Deproject to get 3D distance
	float point0[3], point1[3];
	rs2_deproject_pixel_to_point(point0, &intrinsics, pixel0, dist);
	rs2_deproject_pixel_to_point(point1, &intrinsics, pixel1, dist);
	float rv = sqrt(pow(point1[0]-point0[0], 2)+pow(point1[1]-point0[1], 2)+pow(point1[2]-point0[2], 2));
	pointSize = rv;
}

void MFCamera::stop()
{
	assert(!stopped);
	stopped = true;
	if (grabber_thread) grabber_thread->join();
	if (processing_thread) processing_thread->join();
	if (pipe_started) pipe.stop();
	pipe_started = false;
	processing_done = true;
	processing_done_cv.notify_one();
}

void MFCamera::start_capturer()
{
	assert(stopped);
	stopped = false;
	_start_capture_thread();
	processing_thread = new std::thread(&MFCamera::_processing_thread_main, this);
}

void MFCamera::_start_capture_thread()
{
	grabber_thread = new std::thread(&MFCamera::_capture_thread_main, this);
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
			if (camSettings.do_decimation) processing_frameset = processing_frameset.apply_filter(dec_filter);
			if (camSettings.do_threshold) processing_frameset = processing_frameset.apply_filter(threshold_filter);
			if (camSettings.do_spatial || camSettings.do_temporal) {
				processing_frameset = processing_frameset.apply_filter(depth_to_disparity);
				if (camSettings.do_spatial) processing_frameset = processing_frameset.apply_filter(spat_filter);
				if (camSettings.do_temporal) processing_frameset = processing_frameset.apply_filter(temp_filter);
				processing_frameset = processing_frameset.apply_filter(disparity_to_depth);
			}

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
					if (do_height_filtering && ( pt.y < height_min || pt.y > height_max)) continue;
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
				transformPoint(pt, vertices[i]);
				if (do_height_filtering && (pt.y < height_min || pt.y > height_max)) continue;
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
		if (camData.cloud->size() == 0) {
			std::cerr << "cwipc_realsense2: warning: captured empty pointcloud from camera " << camData.serial << std::endl;
		}
		// Notify wait_for_pc that we're done.
		processing_done = true;
		processing_done_cv.notify_one();
	}
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "frame processing: cam=" << serial << " thread stopped" << std::endl;
#endif
}

void MFCamera::transformPoint(cwipc_pcl_point& out, const rs2::vertex& in)
{
	out.x = (*camData.trafo)(0,0)*in.x + (*camData.trafo)(0,1)*in.y + (*camData.trafo)(0,2)*in.z + (*camData.trafo)(0,3);
	out.y = (*camData.trafo)(1,0)*in.x + (*camData.trafo)(1,1)*in.y + (*camData.trafo)(1,2)*in.z + (*camData.trafo)(1,3);
	out.z = (*camData.trafo)(2,0)*in.x + (*camData.trafo)(2,1)*in.y + (*camData.trafo)(2,2)*in.z + (*camData.trafo)(2,3);
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

