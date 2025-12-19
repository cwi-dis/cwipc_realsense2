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

#include "RS2Camera.hpp"

bool RS2Camera::seek(uint64_t timestamp) {
    return false;
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
