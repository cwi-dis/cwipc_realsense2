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

#include "cwipc_realsense2/private/RS2Config.hpp"
#include "cwipc_realsense2/private/RS2Camera.hpp"


typedef struct HsvColor
{
    unsigned char h;
    unsigned char s;
    unsigned char v;
} HsvColor;

// Helper function - Return HSV color of a point
static HsvColor rgbToHsv(cwipc_pcl_point* pnt)
{
    HsvColor hsv;
    unsigned char rgbMin, rgbMax;

    rgbMin = pnt->r < pnt->g ? (pnt->r < pnt->b ? pnt->r : pnt->b) : (pnt->g < pnt->b ? pnt->g : pnt->b);
    rgbMax = pnt->r > pnt->g ? (pnt->r > pnt->b ? pnt->r : pnt->b) : (pnt->g > pnt->b ? pnt->g : pnt->b);

    hsv.v = rgbMax;
    if (hsv.v == 0)
    {
        hsv.h = 0;
        hsv.s = 0;
        return hsv;
    }

    hsv.s = 255 * ((long)(rgbMax - rgbMin)) / hsv.v;
    if (hsv.s == 0)
    {
        hsv.h = 0;
        return hsv;
    }

    if (rgbMax == pnt->r)
        hsv.h = 0 + 43 * (pnt->g - pnt->b) / (rgbMax - rgbMin);
    else if (rgbMax == pnt->g)
        hsv.h = 85 + 43 * (pnt->b - pnt->r) / (rgbMax - rgbMin);
    else
        hsv.h = 171 + 43 * (pnt->r - pnt->g) / (rgbMax - rgbMin);

    return hsv;
}

// Helper function: return true if point doesn't have to be greenscreen-removed
static bool isNotGreen(cwipc_pcl_point* p)
{
    HsvColor hsv = rgbToHsv(p);

    if (hsv.h >= 60 && hsv.h <= 130) {
        if (hsv.s >= 0.15 && hsv.v >= 0.15) {
            // reducegreen
            if ((p->r * p->b) != 0 && (p->g * p->g) / (p->r * p->b) > 1.5) {
                p->r *= 1.4;
                p->b *= 1.4;
            }
            else {
                p->r *= 1.2;
                p->b *= 1.2;
            }
        }
        return !(hsv.s >= 0.4 && hsv.v >= 0.3);
    }
    return true;
}

// Internal-only constructor for OfflineCamera constructor
RS2Camera::RS2Camera(int _camera_index, rs2::context& ctx, RS2CaptureConfig& configuration, RS2CameraData& _camData)
:	pointSize(0), minx(0), minz(0), maxz(0),
	camera_index(_camera_index),
	serial(_camData.serial),
	stopped(true),
	captured_frame_queue(1),
	camData(_camData),
	camSettings(configuration.camera_config),
    current_pointcloud(nullptr),
	high_speed_connection(true),
	camera_width(0),
	camera_height(0),
	camera_fps(0),
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

RS2Camera::RS2Camera(rs2::context& ctx, RS2CaptureConfig& configuration, int _camera_index, RS2CameraData& _camData, std::string _usb)
:	pointSize(0), minx(0), minz(0), maxz(0),
	camera_index(_camera_index),
	serial(_camData.serial),
	stopped(true),
	captured_frame_queue(1),
	camData(_camData),
	camSettings(configuration.camera_config),
    current_pointcloud(nullptr),
	high_speed_connection(_usb[0] == '3'),
	camera_width(high_speed_connection ? configuration.usb3_width : configuration.usb2_width),
	camera_height(high_speed_connection ? configuration.usb3_height : configuration.usb2_height),
	camera_fps(high_speed_connection ? configuration.usb3_fps : configuration.usb2_fps),
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
		std::cout << "cwipc_realsense2: creating camera " << serial << std::endl;
#endif
    if (!high_speed_connection && !configuration.usb2allowed) {
        cwipc_rs2_log_warning("Camera " + serial + " connected to USB2");
    }
	_init_filters();
}

RS2Camera::~RS2Camera()
{
#ifdef CWIPC_DEBUG
	std::cout << "cwipc_realsense2: destroying " << serial << std::endl;
#endif
	assert(stopped);
}

void RS2Camera::_init_filters()
{
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

void RS2Camera::_init_pointcloud(int size)
{
    if (current_pointcloud == nullptr) {
        current_pointcloud = new_cwipc_pcl_pointcloud();
    }
    current_pointcloud->clear();
    current_pointcloud->reserve(size);

}

bool RS2Camera::capture_frameset()
{
	return captured_frame_queue.try_wait_for_frame(&current_frameset);
}

// Configure and initialize caputuring of one camera
void RS2Camera::start()
{
	assert(stopped);
	rs2::config cfg;
	std::cerr << "cwipc_realsense2: starting camera " << serial << ": " << camera_width << "x" << camera_height << "@" << camera_fps << std::endl;
	cfg.enable_device(serial);
	cfg.enable_stream(RS2_STREAM_COLOR, camera_width, camera_height, RS2_FORMAT_RGB8, camera_fps);
	cfg.enable_stream(RS2_STREAM_DEPTH, camera_width, camera_height, RS2_FORMAT_Z16, camera_fps);
	// xxxjack need to set things like disabling color correction and auto-exposure
	// xxxjack need to allow setting things like laser power
	auto profile = pipe.start(cfg);		// Start streaming with the configuration just set
	_computePointSize(profile);
	pipe_started = true;
}

void RS2Camera::_computePointSize(rs2::pipeline_profile profile)
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
	if (camSettings.do_decimation) {
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

void RS2Camera::stop()
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

void RS2Camera::start_capturer()
{
	assert(stopped);
	stopped = false;
	_start_capture_thread();
	processing_thread = new std::thread(&RS2Camera::_processing_thread_main, this);
	_cwipc_setThreadName(processing_thread, L"cwipc_realsense2::RS2Camera::processing_thread");
}

void RS2Camera::_start_capture_thread()
{
	grabber_thread = new std::thread(&RS2Camera::_capture_thread_main, this);
	_cwipc_setThreadName(grabber_thread, L"cwipc_realsense2::RS2Camera::capture_thread");
}

void RS2Camera::_capture_thread_main()
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

void RS2Camera::_processing_thread_main()
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

        bool do_depth_filtering = camSettings.do_decimation || camSettings.do_threshold || camSettings.do_spatial || camSettings.do_temporal;
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
        if (camSettings.depth_x_erosion >0 || camSettings.depth_y_erosion > 0) {
            _erode_depth(depth, camSettings.depth_x_erosion, camSettings.depth_y_erosion);
        }
#ifdef CWIPC_DEBUG
		std::cerr << "frame processing: cam=" << serial << ", depthseq=" << depth.get_frame_number() << ", colorseq=" << depth.get_frame_number() << std::endl;
#endif

		// Calculate new pointcloud, map to the color images and get vertices and color indices
        pointcloud.map_to(color);
		auto points = pointcloud.calculate(depth);
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
        _init_pointcloud(points.size());

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
                int texture_x = int(0.5 + u*texture_width);
                int texture_y = int(0.5 + v*texture_height);
                // Unsure whether this ever happens: out-of-bounds u/v points are skipped
                if (texture_x <= 0 || texture_x >= texture_width-1) continue;
                if (texture_y <= 0 || texture_y >= texture_height-1) continue;
				int idx = texture_x * texture_x_step + texture_y * texture_y_step;
				pt.r = texture_data[idx];
				pt.g = texture_data[idx + 1];
				pt.b = texture_data[idx + 2];
                // Unexpectedly, this does happen: 100% black points don't actually exist.
                if (pt.r == 0 && pt.g == 0 && pt.b == 0) continue;
				pt.a = camera_label;
				if (!do_greenscreen_removal || isNotGreen(&pt)) // chromakey removal
                    current_pointcloud->push_back(pt);
			}
		}
		if (current_pointcloud->size() == 0) {
			std::cerr << "cwipc_realsense2: warning: captured empty pointcloud from camera " << camData.serial << std::endl;
            //continue;
		}
		// Notify wait_for_pc that we're done.
		processing_done = true;
		processing_done_cv.notify_one();
	}
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "frame processing: cam=" << serial << " thread stopped" << std::endl;
#endif
}

void RS2Camera::_erode_depth(rs2::depth_frame depth_frame, int x_delta, int y_delta)
{
    uint16_t *depth_values = (uint16_t *)depth_frame.get_data();
    assert(depth_frame.get_bytes_per_pixel() == 2);
    int width = depth_frame.get_width();
    int height = depth_frame.get_height();
    int16_t min_depth = (int16_t)(camSettings.threshold_near * 1000);
    int16_t max_depth = (int16_t)(camSettings.threshold_far * 1000);
    int16_t *z_values = (int16_t *)malloc(width * height*sizeof(int16_t));
    // Pass one: Copy Z values to temporary buffer.
    memcpy(z_values, depth_values, width * height*sizeof(int16_t));
    // Pass two: loop for zero pixels in temp buffer, and clear out x/y pixels adjacent in depth buffer
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            if (z_values[x + y*width] != 0) continue;
            // Zero depth at (x, y). Clear out pixels
            for (int ix = x - x_delta; ix <= x + x_delta; ix++) {
                if (ix < 0 || ix >= width ) continue;
                int i_pc = (ix + y * width);
                depth_values[i_pc] = 0;
            }
            for (int iy = y - y_delta; iy <= y + y_delta; iy++) {
                if (iy < 0 || iy >= height) continue;
                int i_pc = (x + iy * width);
                depth_values[i_pc] = 0;
            }
        }
    }
    free(z_values);
}

void RS2Camera::transformPoint(cwipc_pcl_point& out, const rs2::vertex& in)
{
	out.x = (*camData.trafo)(0,0)*in.x + (*camData.trafo)(0,1)*in.y + (*camData.trafo)(0,2)*in.z + (*camData.trafo)(0,3);
	out.y = (*camData.trafo)(1,0)*in.x + (*camData.trafo)(1,1)*in.y + (*camData.trafo)(1,2)*in.z + (*camData.trafo)(1,3);
	out.z = (*camData.trafo)(2,0)*in.x + (*camData.trafo)(2,1)*in.y + (*camData.trafo)(2,2)*in.z + (*camData.trafo)(2,3);
}

void RS2Camera::create_pc_from_frames()
{
	processing_frame_queue.enqueue(current_frameset);
}

void RS2Camera::wait_for_pc()
{
	std::unique_lock<std::mutex> lock(processing_mutex);
	processing_done_cv.wait(lock, [this]{ return processing_done; });
	processing_done = false;
}

uint64_t RS2Camera::get_capture_timestamp()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void
RS2Camera::save_auxdata(cwipc *pc, bool rgb, bool depth)
{
    if (rgb) {
        std::string name = "rgb." + serial;
        rs2::video_frame color = current_frameset.get_color_frame();
        //const void* pointer = color.get_data();
        const size_t size = color.get_data_size();
        void* pointer = malloc(size);
        if (pointer) {
            memcpy(pointer, color.get_data(), size);
            cwipc_auxiliary_data *ap = pc->access_auxiliary_data();
            ap->_add(name, pointer, size, ::free);
        }
    }
    if (depth) {
        std::string name = "depth." + serial;
        rs2::video_frame depth = current_frameset.get_depth_frame();
        //const void* pointer = depth.get_data();
        const size_t size = depth.get_data_size();
        void* pointer = malloc(size);
        if (pointer) {
            memcpy(pointer, depth.get_data(), size);
            cwipc_auxiliary_data *ap = pc->access_auxiliary_data();
            ap->_add(name, pointer, size, ::free);
        }
    }
}

