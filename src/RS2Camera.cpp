//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//
#include <cstdlib>

// Define to get (a little) debug prints
#undef CWIPC_DEBUG
#undef CWIPC_DEBUG_THREAD
#define CWIPC_DEBUG_SYNC

// Only for RGB and Depth auxdata: we have the option of mapping depth to color or color to depth.
#define MAP_DEPTH_IMAGE_TO_COLOR_IMAGE

// This is the dll source, so define external symbols as dllexport on windows.

#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif

#include <librealsense2/rsutil.h>

#include "cwipc_realsense2/private/RS2Config.hpp"
#include "cwipc_realsense2/private/RS2Camera.hpp"

typedef struct HsvColor {
    unsigned char h;
    unsigned char s;
    unsigned char v;
} HsvColor;

// Helper function - Return HSV color of a point
static HsvColor rgbToHsv(cwipc_pcl_point* pnt) {
    HsvColor hsv;
    unsigned char rgbMin, rgbMax;

    rgbMin = pnt->r < pnt->g ? (pnt->r < pnt->b ? pnt->r : pnt->b) : (pnt->g < pnt->b ? pnt->g : pnt->b);
    rgbMax = pnt->r > pnt->g ? (pnt->r > pnt->b ? pnt->r : pnt->b) : (pnt->g > pnt->b ? pnt->g : pnt->b);

    hsv.v = rgbMax;
    if (hsv.v == 0) {
        hsv.h = 0;
        hsv.s = 0;
        return hsv;
    }

    hsv.s = 255 * ((long)(rgbMax - rgbMin)) / hsv.v;
    if (hsv.s == 0) {
        hsv.h = 0;
        return hsv;
    }

    if (rgbMax == pnt->r) {
        hsv.h = 0 + 43 * (pnt->g - pnt->b) / (rgbMax - rgbMin);
    } else if (rgbMax == pnt->g) {
        hsv.h = 85 + 43 * (pnt->b - pnt->r) / (rgbMax - rgbMin);
    } else {
        hsv.h = 171 + 43 * (pnt->r - pnt->g) / (rgbMax - rgbMin);
    }

    return hsv;
}

// Helper function: return true if point doesn't have to be greenscreen-removed
static bool isNotGreen(cwipc_pcl_point* p) {
    HsvColor hsv = rgbToHsv(p);

    if (hsv.h >= 60 && hsv.h <= 130) {
        if (hsv.s >= 0.15 && hsv.v >= 0.15) {
            // reducegreen
            if ((p->r * p->b) != 0 && (p->g * p->g) / (p->r * p->b) > 1.5) {
                p->r *= 1.4;
                p->b *= 1.4;
            } else {
                p->r *= 1.2;
                p->b *= 1.2;
            }
        }

        return !(hsv.s >= 0.4 && hsv.v >= 0.3);
    }

    return true;
}


inline bool isPointInRadius(cwipc_pcl_point& pt, float radius_filter) {
    float distance_2 = pow(pt.x, 2) + pow(pt.z, 2);
    return distance_2 < radius_filter * radius_filter; // radius^2 to avoid sqrt
}

RS2Camera::RS2Camera(rs2::context& _ctx, RS2CaptureConfig& configuration, int _camera_index) :
  pointSize(0), 
#if 0
  minx(0), minz(0), maxz(0),
#endif
  camera_index(_camera_index),
  serial(configuration.all_camera_configs[_camera_index].serial),
  camera_stopped(true),
  camera_config(configuration.all_camera_configs[_camera_index]),
  filtering(configuration.filtering),
  processing(configuration.processing),
  hardware(configuration.hardware),
  auxData(configuration.auxData),
  current_pointcloud(nullptr),
  processing_frame_queue(1),
  capturer_context(_ctx),
  camera_pipeline(_ctx),
  camera_pipeline_started(false),
  align_color_to_depth(RS2_STREAM_DEPTH),
  align_depth_to_color(RS2_STREAM_COLOR),
  debug(configuration.debug),
  prefer_color_timing(configuration.prefer_color_timing)
{
#ifdef CWIPC_DEBUG
    if (debug) std::cout << "cwipc_realsense2: creating camera " << serial << std::endl;
#endif

    if (configuration.record_to_directory != "") {
        record_to_file = configuration.record_to_directory + "/" + serial + ".bag";
    }

    _init_filters();
}

RS2Camera::~RS2Camera() {
#ifdef CWIPC_DEBUG
    if (debug) std::cout << "cwipc_realsense2: destroying " << serial << std::endl;
#endif
    assert(camera_stopped);
}

bool RS2Camera::getHardwareParameters(RS2CameraHardwareConfig& output, bool match) {
    if (match) {
        return 
            output.fps == hardware.fps &&

            output.color_height == hardware.color_height &&
            output.color_width == hardware.color_width &&
            output.color_exposure == hardware.color_exposure &&
            output.color_gain == hardware.color_gain &&
            output.backlight_compensation == hardware.backlight_compensation &&
            output.whitebalance == hardware.whitebalance &&

            output.depth_exposure == hardware.depth_exposure &&
            output.depth_gain == hardware.depth_gain &&
            output.depth_height == hardware.depth_height &&
            output.depth_width == hardware.depth_width &&
            output.visual_preset == hardware.visual_preset &&
            output.laser_power == hardware.laser_power;
    }
    output = hardware;
    return output.fps != 0;
}

void RS2Camera::_init_filters() {
    if (filtering.do_decimation) {
        dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, filtering.decimation_magnitude);
    }

    if (filtering.do_threshold) {
        threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, filtering.threshold_min_distance);
        threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, filtering.threshold_max_distance);
    }

    if (filtering.do_spatial) {
        spatial_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, filtering.spatial_magnitude);
        spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, filtering.spatial_smooth_alpha);
        spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, filtering.spatial_smooth_delta);
        spatial_filter.set_option(RS2_OPTION_HOLES_FILL, filtering.spatial_holes_fill);
    }

    if (filtering.do_temporal) {
        temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, filtering.temporal_smooth_alpha);
        temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, filtering.temporal_smooth_delta);
        // Note by Jack: the following is correct. The temporal persistency is set with the HOLES_FILL parameter...
        temporal_filter.set_option(RS2_OPTION_HOLES_FILL, filtering.temporal_persistency);
    }
    if (filtering.do_hole_filling) {
        hole_filling_filter.set_option(RS2_OPTION_HOLES_FILL, filtering.hole_filling_mode);
    }
}

void RS2Camera::_init_current_pointcloud(int size) {
    if (current_pointcloud == nullptr) {
        current_pointcloud = new_cwipc_pcl_pointcloud();
    }

    current_pointcloud->clear();
    current_pointcloud->reserve(size);
}

uint64_t RS2Camera::wait_for_captured_frameset(uint64_t minimum_timestamp) {
    uint64_t resultant_timestamp = 0;
    if (minimum_timestamp > 0 && previous_captured_frameset) {
        uint64_t previous_timestamp = previous_captured_frameset.get_depth_frame().get_timestamp();
        if (previous_timestamp > minimum_timestamp) {
            current_captured_frameset = previous_captured_frameset;
#ifdef CWIPC_DEBUG_SYNC
            if (debug) {
                uint64_t depth_timestamp = current_captured_frameset.get_depth_frame().get_timestamp();
                uint64_t color_timestamp = current_captured_frameset.get_color_frame().get_timestamp();
                std::cerr << "wait_for_captured_frameset: cam=" << camera_index << ", dts=" << depth_timestamp << ", cts=" << color_timestamp << " (previous)" << std::endl;
            }
#endif
            return previous_timestamp;
        }
    }
    do {
        if (current_captured_frameset) {
            previous_captured_frameset = current_captured_frameset;
        } else {
            // First frame. So fill previous_captured_frameset.
            previous_captured_frameset = wait_for_frames();
        }
        current_captured_frameset = wait_for_frames();
        // We now have both a previous and a current captured frameset.
        if(prefer_color_timing) {
            // If they both have the same depth timestamp we capture another one.
            if (current_captured_frameset.get_depth_frame().get_timestamp() == previous_captured_frameset.get_depth_frame().get_timestamp()) {
                previous_captured_frameset = current_captured_frameset;
                current_captured_frameset = wait_for_frames();
            }
        }
        rs2::depth_frame depth_frame = current_captured_frameset.get_depth_frame();
        resultant_timestamp = (uint64_t)depth_frame.get_timestamp();
    } while(resultant_timestamp < minimum_timestamp);
#ifdef CWIPC_DEBUG_SYNC
    if (debug) {
        uint64_t depth_timestamp = current_captured_frameset.get_depth_frame().get_timestamp();
        uint64_t color_timestamp = current_captured_frameset.get_color_frame().get_timestamp();
        std::cerr << "wait_for_captured_frameset: cam=" << camera_index << ", dts=" << depth_timestamp << ", cts=" << color_timestamp << std::endl;
    }
#endif
    return resultant_timestamp;
}

// Configure and initialize caputuring of one camera
void RS2Camera::start_camera() {
    assert(camera_stopped);
    assert(!camera_pipeline_started);
    assert(camera_capture_thread == nullptr);
    assert(camera_processing_thread == nullptr);
    rs2::config cfg;
#ifdef CWIPC_DEBUG
    if (debug) std::cerr << "cwipc_realsense2: starting camera " << serial << ": " << camera_width << "x" << camera_height << "@" << camera_fps << std::endl;
#endif
    _pre_start(cfg);
    rs2::pipeline_profile profile = camera_pipeline.start(cfg);   // Start streaming with the configuration just set
    _post_start(profile);
    _computePointSize(profile);
    camera_pipeline_started = true;
}

void RS2Camera::_pre_start(rs2::config &cfg) {
    cfg.enable_device(serial);
    uses_recorder = record_to_file != "";
    if (uses_recorder) {
        std::cerr << "RS2Camera::_pre_start: recording to " << record_to_file << std::endl;
        cfg.enable_record_to_file(record_to_file);
    }
    cfg.enable_stream(RS2_STREAM_COLOR, hardware.color_width, hardware.color_height, RS2_FORMAT_RGB8, hardware.fps);
    cfg.enable_stream(RS2_STREAM_DEPTH, hardware.depth_width, hardware.depth_height, RS2_FORMAT_Z16, hardware.fps);
}

void RS2Camera::_post_start(rs2::pipeline_profile& profile) {
    rs2::device dev = profile.get_device();
    // First pause the recorder, if there is one.
    if (uses_recorder) {
        rs2::recorder recorder = dev.as<rs2::recorder>();
        if (!recorder) {
            std::cerr << "RS2Camera::_post_start: uses_recorder but no recorder" << std::endl;
        } else {
            recorder.pause();
        }
    }
    // Obtain actual serial number and fps. Most important for playback cameras, but also useful for
    // live cameras (because the user program can obtain correct cameraconfig data with get_config()).
    // Keep actual serial number, also in cameraconfig
    serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    camera_config.serial = serial;
    std::vector<rs2::sensor> sensors = dev.query_sensors();
    int depth_fps = hardware.fps;
    int depth_width = hardware.depth_width;
    int depth_height = hardware.depth_height;
    int color_fps = hardware.fps;
    int color_width = hardware.color_width;
    int color_height = hardware.color_height;
    for(rs2::sensor sensor : sensors) {
        std::vector<rs2::stream_profile> streams = sensor.get_active_streams();
        for(rs2::stream_profile stream : streams) {
            rs2::video_stream_profile vstream = stream.as<rs2::video_stream_profile>();

            if (vstream.stream_type() == RS2_STREAM_DEPTH) {
                depth_fps = vstream.fps();
                depth_width = vstream.width();
                depth_height = vstream.height();
                depth_format = rs2_format_to_string(vstream.format());

            }
            if (stream.stream_type() == RS2_STREAM_COLOR) {
                color_fps = vstream.fps();
                color_width = vstream.width();
                color_height = vstream.height();
                color_format = rs2_format_to_string(vstream.format());
            }
        }
    }
    if (depth_fps != color_fps) {
        std::cerr << "RS2Camera: Warning: depth_fps=" << depth_fps << " and color_fps=" << color_fps << std::endl;
    }
    hardware.fps = depth_fps;
    hardware.depth_width = depth_width;
    hardware.depth_height = depth_height;
    hardware.color_width = color_width;
    hardware.color_height = color_height;
}

void RS2Camera::post_start_all_cameras() {
    // First pause the recorder, if there is one.
    if (uses_recorder) {
        rs2::pipeline_profile profile = camera_pipeline.get_active_profile();
        rs2::device dev = profile.get_device();
        rs2::recorder recorder = dev.as<rs2::recorder>();
        if (!recorder) {
            std::cerr << "RS2Camera::post_start_all_cameras: uses_recorder but no recorder" << std::endl;
        } else {
            recorder.resume();
        }
    } 
}

void RS2Camera::_computePointSize(rs2::pipeline_profile profile) {
    // Get the 3D distance between camera and (0,0,0) or use 1m if unreasonable
    float tx = (*camera_config.trafo)(0,3);
    float ty = (*camera_config.trafo)(1,3);
    float tz = (*camera_config.trafo)(2,3);
    float dist = sqrt(tx*tx + ty*ty + tz*tz);

    if (dist == 0) {
        dist = 1;
    }

    // Now get the intrinsics for the depth stream
    auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto intrinsics = stream.get_intrinsics(); // Calibration data

    // Compute 2D coordinates of adjacent pixels in the middle of the field of view
    float pixel0[2], pixel1[2];
    pixel0[0] = hardware.depth_width / 2;
    pixel0[1] = hardware.depth_height / 2;
    if (filtering.do_decimation) {
        pixel1[0] = pixel0[0] + filtering.decimation_magnitude;
        pixel1[1] = pixel0[1] + filtering.decimation_magnitude;
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

void RS2Camera::pre_stop_camera() {
    if (uses_recorder) {
        rs2::pipeline_profile profile = camera_pipeline.get_active_profile();
        rs2::device dev = profile.get_device();
        rs2::recorder recorder = dev.as<rs2::recorder>();
        if (!recorder) {
            std::cerr << "RS2Camera::post_start_all_cameras: uses_recorder but no recorder" << std::endl;
        } else {
            recorder.pause();
        }
    }
}

void RS2Camera::stop_camera() {
    // Tell the grabber and processor thread that we want to stop.
    assert(!camera_stopped);
    camera_stopped = true;

    // Clear out the queues so we don't get into a deadlock
    
    while (true) {
        rs2::frameset tmp;
        bool ok = processing_frame_queue.try_wait_for_frame(&tmp, 1);
        if (!ok) break;
    }
    
    // Join and delete the processor thread
    if (camera_processing_thread) {
        camera_processing_thread->join();
    }

    delete camera_processing_thread;
    camera_processing_thread = nullptr;

    // stop the pipeline.
    if (camera_pipeline_started) {
        camera_pipeline.stop();
    }

    camera_pipeline_started = false;
    processing_done = true;
    processing_done_cv.notify_one();
}

void RS2Camera::start_capturer() {
    assert(camera_stopped);
    camera_stopped = false;

    _start_processing_thread();
}

void RS2Camera::_start_processing_thread() {
    camera_processing_thread = new std::thread(&RS2Camera::_processing_thread_main, this);
    _cwipc_setThreadName(camera_processing_thread, L"cwipc_realsense2::RS2Camera::processing_thread");
}

#if 0

int64_t RS2Camera::_frameset_timedelta_preferred(rs2::frameset frames) {
    if (preferred_timestamp == 0) return 0;
    rs2::frame depth_frame = frames.get_depth_frame();
    uint64_t frame_timestamp = (uint64_t)depth_frame.get_timestamp();
    int64_t delta = preferred_timestamp - frame_timestamp;
    int64_t frame_duration = (1000 / hardware.fps);
    if (delta < -frame_duration) {
        // frame_timestamp > preferred timestamp, so frame is more than one frametime in the future
#ifdef CWIPC_DEBUG_SYNCext
        std::cerr << "xxxjack cam=" << camera_index << ", preferred=" << preferred_timestamp <<", frame is " << (-delta) << " in the future" << std::endl;
#endif
        return delta;
    } else if( delta > 1 /* ms */) {
        // frame_timestamp > preferred_timestamp, so frame is in the past (by more than half a frame)
#ifdef CWIPC_DEBUG_SYNCext
        std::cerr << "xxxjack cam=" << camera_index << ", preferred=" << preferred_timestamp <<", frame is " << (delta) << " in the past" << std::endl;
#endif
        return delta;
    }
    return 0;
}
#endif

rs2::frameset RS2Camera::wait_for_frames() {
    rs2::frameset frames = camera_pipeline.wait_for_frames();
#ifdef CWIPC_DEBUG_SYNC
    if (debug) {
        uint64_t depth_timestamp = frames.get_depth_frame().get_timestamp();
        uint64_t color_timestamp = frames.get_color_frame().get_timestamp();
        std::cerr << "wait_for_frames: cam=" << camera_index << ", dts=" << depth_timestamp << ", cts=" << color_timestamp << std::endl;
    }
#endif
    return frames;
}

void RS2Camera::_processing_thread_main() {
#ifdef CWIPC_DEBUG_THREAD
    if (debug) std::cerr << "frame processing: cam=" << camera_index << " thread started" << std::endl;
#endif

    while(!camera_stopped) {
        // Wait for next frame to process. Allow aborting in case of stopped becoming false...
        rs2::frameset processing_frameset;
        bool ok = processing_frame_queue.try_wait_for_frame(&processing_frameset, 1000);
        if (!ok) {
            continue;
        }
#ifdef CWIPC_DEBUG_SYNC
    if (debug) {
        uint64_t depth_timestamp = processing_frameset.get_depth_frame().get_timestamp();
        uint64_t color_timestamp = processing_frameset.get_color_frame().get_timestamp();
        std::cerr << "processing_thread: cam=" << camera_index << ", dts=" << depth_timestamp << ", cts=" << color_timestamp << std::endl;
    }
#endif

        std::lock_guard<std::mutex> lock(processing_mutex);

        bool do_depth_filtering = filtering.do_decimation || filtering.do_threshold || filtering.do_spatial || filtering.do_temporal;
        if (filtering.map_color_to_depth == 1) {
            processing_frameset = processing_frameset.apply_filter(align_color_to_depth);
        } else if (filtering.map_color_to_depth == 0) {
            processing_frameset = processing_frameset.apply_filter(align_depth_to_color);
        } else {
            // map_color_to_depth == -1 means no mapping t all.
        }
        if (do_depth_filtering) {
          
            if (filtering.do_decimation) {
                processing_frameset = processing_frameset.apply_filter(dec_filter);
            }

            if (filtering.do_threshold) {
                processing_frameset = processing_frameset.apply_filter(threshold_filter);
            }

            if (filtering.do_spatial || filtering.do_temporal || filtering.do_hole_filling) {
                processing_frameset = processing_frameset.apply_filter(depth_to_disparity);

                if (filtering.do_spatial) {
                    processing_frameset = processing_frameset.apply_filter(spatial_filter);
                }

                if (filtering.do_temporal) {
                    processing_frameset = processing_frameset.apply_filter(temporal_filter);
                }
                if (filtering.do_hole_filling) {
                    processing_frameset = processing_frameset.apply_filter(hole_filling_filter);
                }

                processing_frameset = processing_frameset.apply_filter(disparity_to_depth);
            }

        }
        current_processed_frameset = processing_frameset;

        rs2::depth_frame depth = processing_frameset.get_depth_frame();
        rs2::video_frame color = processing_frameset.get_color_frame();
        
        assert(depth);
        assert(color);
#if xxxjack_dont_want
        // This code is disabled, because here we get the size of the processed
        // images, not the images as captured.
        int color_width = color.get_width();
        int color_height = color.get_height();
        int depth_width = depth.get_width();
        int depth_height = depth.get_height();

        // compare width/height to what the frames have.
        if(hardware.color_width != 0 && hardware.color_width != color_width) {
            std::cerr << "RS2Camera: color frame width " << color_width << ", expected " << hardware.color_width << std::endl;
        }
        if(hardware.color_height != 0 && hardware.color_height != color_height) {
            std::cerr << "RS2Camera: color frame height " << color_height << ", expected " << hardware.color_height << std::endl;
        }
        if(hardware.depth_width != 0 && hardware.depth_width != depth_width) {
            std::cerr << "RS2Camera: depth frame width " << depth_width << ", expected " << hardware.depth_width << std::endl;
        }
        if(hardware.depth_height != 0 && hardware.depth_height != depth_height) {
            std::cerr << "RS2Camera: depth frame height " << depth_height << ", expected " << hardware.depth_height << std::endl;
        }
        hardware.color_width = color_width;
        hardware.color_height = color_height;
        hardware.depth_width = depth_width;
        hardware.depth_height = depth_height;
#endif

        if (processing.depth_x_erosion > 0 || processing.depth_y_erosion > 0) {
            _erode_depth(depth, processing.depth_x_erosion, processing.depth_y_erosion);
        }
#ifdef CWIPC_DEBUG
        if (debug) std::cerr << "frame processing: cam=" << camera_index << ", depthseq=" << depth.get_frame_number() << ", colorseq=" << depth.get_frame_number() << std::endl;
#endif

        // Calculate new pointcloud, map to the color images and get vertices and color indices
        depth_to_pointcloud.map_to(color);
        rs2::points points = depth_to_pointcloud.calculate(depth);
        const rs2::vertex* vertices = points.get_vertices();
        const rs2::texture_coordinate* texture_coordinates = points.get_texture_coordinates();

        // Get some constants used later to map colors and such from rs2 to pcl pointclouds.
        const int texture_width = color.get_width();
        const int texture_height = color.get_height();
        const int texture_x_step = color.get_bytes_per_pixel();
        const int texture_y_step = color.get_stride_in_bytes();
        const unsigned char *texture_data = (unsigned char*)color.get_data();
        const uint8_t camera_label = (uint8_t)1 << camera_index;

        // Clear the previous pointcloud and pre-allocate space in the pointcloud (so we don't realloc)
        _init_current_pointcloud((int)points.size());
        cwipc_pcl_pointcloud pcl_pointcloud = current_pointcloud;

        {
            // Make PointCloud
            float height_min = processing.height_min;
            float height_max = processing.height_max;
            bool do_height_filtering = height_min < height_max;
            bool do_greenscreen_removal = processing.greenscreen_removal;
            bool do_radius_filtering = processing.radius_filter > 0;
            for (int i = 0; i < points.size(); i++) {
                // Skip points with z=0 (they don't exist)
                if (vertices[i].z == 0) {
                    continue;
                }

                cwipc_pcl_point pt;
                transformPoint(pt, vertices[i]);

                if (do_height_filtering && (pt.y < height_min || pt.y > height_max)) {
                    continue;
                }
                if (do_radius_filtering && !isPointInRadius(pt, processing.radius_filter)) {
                    continue;
                }
                float u = texture_coordinates[i].u;
                float v = texture_coordinates[i].v;

                int texture_x = int(0.5 + u*texture_width);
                int texture_y = int(0.5 + v*texture_height);

                // Unsure whether this ever happens: out-of-bounds u/v points are skipped
                if (texture_x <= 0 || texture_x >= texture_width-1) {
                    continue;
                }

                if (texture_y <= 0 || texture_y >= texture_height-1) {
                    continue;
                }

                int idx = texture_x * texture_x_step + texture_y * texture_y_step;
                pt.r = texture_data[idx];
                pt.g = texture_data[idx + 1];
                pt.b = texture_data[idx + 2];

                // Unexpectedly, this does happen: 100% black points don't actually exist.
                if (pt.r == 0 && pt.g == 0 && pt.b == 0) {
                    continue;
                }

                pt.a = camera_label;

                if (!do_greenscreen_removal || isNotGreen(&pt)) {
                    // chromakey removal
                    pcl_pointcloud->push_back(pt);
                }
            }
        }

        if (pcl_pointcloud->size() == 0) {
#ifdef CWIPC_DEBUG
          if (debug) std::cerr << "cwipc_realsense2: warning: captured empty pointcloud from camera " << camera_config.serial << std::endl;
#endif
          //continue;
        }

        // Notify wait_for_pc that we're done.
        processing_done = true;
        processing_done_cv.notify_one();
    }

#ifdef CWIPC_DEBUG_THREAD
    if (debug) std::cerr << "frame processing: cam=" << camera_index << " thread stopped" << std::endl;
#endif
}

void RS2Camera::_erode_depth(rs2::depth_frame depth_frame, int x_delta, int y_delta) {
    uint16_t *depth_values = (uint16_t *)depth_frame.get_data();
    assert(depth_frame.get_bytes_per_pixel() == 2);
    int width = depth_frame.get_width();
    int height = depth_frame.get_height();
    int16_t min_depth = (int16_t)(filtering.threshold_min_distance * 1000);
    int16_t max_depth = (int16_t)(filtering.threshold_max_distance * 1000);
    int16_t *z_values = (int16_t *)malloc(width * height*sizeof(int16_t));

    // Pass one: Copy Z values to temporary buffer.
    memcpy(z_values, depth_values, width * height*sizeof(int16_t));

    // Pass two: loop for zero pixels in temp buffer, and clear out x/y pixels adjacent in depth buffer
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            if (z_values[x + y*width] != 0) {
               continue;
            }

            // Zero depth at (x, y). Clear out pixels
            for (int ix = x - x_delta; ix <= x + x_delta; ix++) {
                if (ix < 0 || ix >= width) {
                    continue;
                }

                int i_pc = (ix + y * width);
                depth_values[i_pc] = 0;
            }
            for (int iy = y - y_delta; iy <= y + y_delta; iy++) {
                if (iy < 0 || iy >= height) {
                    continue;
                }

                int i_pc = (x + iy * width);
                depth_values[i_pc] = 0;
            }
        }
    }

    free(z_values);
}

void RS2Camera::transformPoint(cwipc_pcl_point& out, const rs2::vertex& in) {
    out.x = (*camera_config.trafo)(0,0)*in.x + (*camera_config.trafo)(0,1)*in.y + (*camera_config.trafo)(0,2)*in.z + (*camera_config.trafo)(0,3);
    out.y = (*camera_config.trafo)(1,0)*in.x + (*camera_config.trafo)(1,1)*in.y + (*camera_config.trafo)(1,2)*in.z + (*camera_config.trafo)(1,3);
    out.z = (*camera_config.trafo)(2,0)*in.x + (*camera_config.trafo)(2,1)*in.y + (*camera_config.trafo)(2,2)*in.z + (*camera_config.trafo)(2,3);
}

void RS2Camera::transformPoint(float out[3], const float in[3]) {
    out[0] = (*camera_config.trafo)(0,0)*in[0] + (*camera_config.trafo)(0,1)*in[1] + (*camera_config.trafo)(0,2)*in[2] + (*camera_config.trafo)(0,3);
    out[1] = (*camera_config.trafo)(1,0)*in[0] + (*camera_config.trafo)(1,1)*in[1] + (*camera_config.trafo)(1,2)*in[2] + (*camera_config.trafo)(1,3);
    out[2] = (*camera_config.trafo)(2,0)*in[0] + (*camera_config.trafo)(2,1)*in[1] + (*camera_config.trafo)(2,2)*in[2] + (*camera_config.trafo)(2,3);
}

void RS2Camera::create_pc_from_frameset() {
#ifdef CWIPC_DEBUG_SYNC
    if (debug) {
        uint64_t depth_timestamp = current_captured_frameset.get_depth_frame().get_timestamp();
        uint64_t color_timestamp = current_captured_frameset.get_color_frame().get_timestamp();
        std::cerr << "create_pc_from_frameset: cam=" << camera_index << ", dts=" << depth_timestamp << ", cts=" << color_timestamp << std::endl;
    }
#endif
    processing_frame_queue.enqueue(current_captured_frameset);
}

void RS2Camera::wait_for_pc_created() {
    std::unique_lock<std::mutex> lock(processing_mutex);
    processing_done_cv.wait(lock, [this]{ return processing_done; });
    processing_done = false;
}

bool RS2Camera::map2d3d(int x_2d, int y_2d, int d_2d, float *out3d)
{
    float tmp3d[3] = {0, 0, 0};
#if 0
    float in2d[2] = {
        (float)x_2d,
        (float)y_2d
    };
    // Note by Jack: the 1000.0 is a magic value, somewhere it is stated that the D4xx cameras use 1mm as the
    // depth unit. xxxjack it should use get_depth_scale().
    float indepth = float(d_2d) / 1000.0;
    
#if 0
    // Now get the intrinsics for the depth stream
    rs2::pipeline_profile pipeline_profile = camera_pipeline.get_active_profile();
    rs2::video_stream_profile stream_profile = (
        filtering.map_color_to_depth != 0 ?
        pipeline_profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>() :
        pipeline_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>()
    );
#else
    rs2::video_stream_profile stream_profile = current_processed_frameset.get_depth_frame().get_profile().as<rs2::video_stream_profile>();
    if (stream_profile == nullptr) {
        std::cerr << "RS2Camera: map2d3d: cannot get stream_profile for intrinsics" << std::endl;
        return false;
    }
#endif
    rs2_intrinsics intrinsics = stream_profile.get_intrinsics(); // Calibration data
    rs2_deproject_pixel_to_point(tmp3d, &intrinsics, in2d, indepth);
#else
    // Different route. We look up the depth value.
    // First we need to get the four matrices.
    rs2::depth_frame depth_frame = current_processed_frameset.get_depth_frame();
    rs2::video_frame color_frame = current_processed_frameset.get_color_frame();
    rs2::video_stream_profile depth_profile = depth_frame.get_profile().as<rs2::video_stream_profile>();
    rs2::video_stream_profile color_profile = color_frame.get_profile().as<rs2::video_stream_profile>();
    rs2_intrinsics depth_intrinsics = depth_profile.get_intrinsics();
    rs2_intrinsics color_intrinsics = color_profile.get_intrinsics();
    rs2_extrinsics depth_to_color = depth_profile.get_extrinsics_to(color_profile);
    rs2_extrinsics color_to_depth = color_profile.get_extrinsics_to(depth_profile);
    const uint16_t* depth_data = (const uint16_t *)depth_frame.get_data();
    // Now we can convert the RGB x,y coordinates to depth x,y coordinates.
    float xy_color[2] { 
        float(x_2d), 
        float(y_2d)
        };
    float xy_depth[2];
    float min_distance = filtering.threshold_min_distance;
    float max_distance = filtering.threshold_max_distance;
    rs2_project_color_pixel_to_depth_pixel(
        xy_depth,
        depth_data,
        1000,
        min_distance,
        max_distance,
        &depth_intrinsics,
        &color_intrinsics,
        &color_to_depth,
        &depth_to_color,
        xy_color
    );
    // Now we can use uv_depth to find the correct depth value.
    uint16_t depth_i = depth_data[(int)xy_depth[1]*depth_intrinsics.width + (int)xy_depth[0]];
    rs2_deproject_pixel_to_point(tmp3d, &depth_intrinsics, xy_depth, depth_i / 1000.0);
#endif
    transformPoint(out3d, tmp3d);
    return true;
}

void RS2Camera::save_frameset_auxdata(cwipc *pc)
{
    if (!auxData.want_auxdata_depth && !auxData.want_auxdata_rgb && !auxData.want_image_timestamps) return;
    std::unique_lock<std::mutex> lock(processing_mutex);

    auto aligned_frameset = current_processed_frameset;
    if (aligned_frameset.size() == 0) return;
    rs2::video_frame color_image = aligned_frameset.get_color_frame();
    rs2::video_frame depth_image = aligned_frameset.get_depth_frame();
        
    if (auxData.want_image_timestamps) {
        
        int64_t depth_framenum = depth_image.get_frame_number();
        int64_t depth_timestamp = depth_image.get_timestamp();
        int32_t depth_clock = depth_image.get_frame_timestamp_domain();
        int64_t color_framenum = color_image.get_frame_number();
        int64_t color_timestamp = color_image.get_timestamp();
        int32_t color_clock = color_image.get_frame_timestamp_domain();
        std::string timestamp_data = 
            "depth_framenum=" + std::to_string(depth_framenum) +
            ",depth_timestamp=" + std::to_string(depth_timestamp) +
            ",depth_clock=" + std::to_string(depth_clock) +
            ",color_framenum=" + std::to_string(color_framenum) +
            ",color_timestamp=" + std::to_string(color_timestamp) +
            ",color_clock=" + std::to_string(color_clock);
        rs2::playback playback = camera_pipeline.get_active_profile().get_device().as<rs2::playback>();
        if (playback) {
            timestamp_data += ",filetime_ns=" + std::to_string(playback.get_position());
        }
        cwipc_auxiliary_data *ap = pc->access_auxiliary_data();
        std::string name = "timestamps." + serial;
        ap->_add(name, timestamp_data, nullptr, 0, ::free);
    }
    if (auxData.want_auxdata_rgb) {
        std::string name = "rgb." + serial;
        //const void* pointer = color.get_data();
        const size_t size = color_image.get_data_size();
        int width = color_image.get_width();
        int height = color_image.get_height();
        int stride = color_image.get_stride_in_bytes();
        int bpp = color_image.get_bytes_per_pixel();

        std::string description =
            "width="+std::to_string(width)+
            ",height="+std::to_string(height)+
            ",stride="+std::to_string(stride)+
            ",bpp="+std::to_string(bpp)+
            ",format="+std::string(color_format);

        void* pointer = malloc(size);

        if (pointer) {
            memcpy(pointer, color_image.get_data(), size);
            cwipc_auxiliary_data *ap = pc->access_auxiliary_data();
            ap->_add(name, description, pointer, size, ::free);
        }
    }

    if (auxData.want_auxdata_depth) {
        std::string name = "depth." + serial;
        const size_t size = depth_image.get_data_size();
        int width = depth_image.get_width();
        int height = depth_image.get_height();
        int stride = depth_image.get_stride_in_bytes();
        int bpp = depth_image.get_bytes_per_pixel();

        std::string description =
            "width="+std::to_string(width)+
            ",height="+std::to_string(height)+
            ",stride="+std::to_string(stride)+
            ",bpp="+std::to_string(bpp)+
            "format="+std::string(color_format);

        void* pointer = malloc(size);

        if (pointer) {
            memcpy(pointer, depth_image.get_data(), size);
            cwipc_auxiliary_data *ap = pc->access_auxiliary_data();
            ap->_add(name, description, pointer, size, ::free);
        }
    }
}
