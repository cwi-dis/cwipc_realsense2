// Define to get (a little) debug prints
#undef CWIPC_DEBUG
#undef CWIPC_DEBUG_THREAD
#define CWIPC_DEBUG_SYNC

// Only for RGB and Depth metadata: we have the option of mapping depth to color or color to depth.
#define MAP_DEPTH_IMAGE_TO_COLOR_IMAGE

// This is the dll source, so define external symbols as dllexport on windows.

#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif

#include <librealsense2/rsutil.h>

#include "RS2Camera.hpp"

void RS2Camera::_init_pipeline_for_this_camera(rs2::config &cfg) {
    cfg.enable_device(get_serial());
    _uses_recorder = _record_to_file != "";
    if (_uses_recorder) {
        _log_trace("enabling recorder to file " + _record_to_file);
        cfg.enable_record_to_file(_record_to_file);
    }
    cfg.enable_stream(RS2_STREAM_COLOR, _hardware.color_width, _hardware.color_height, RS2_FORMAT_RGB8, _hardware.fps);
    cfg.enable_stream(RS2_STREAM_DEPTH, _hardware.depth_width, _hardware.depth_height, RS2_FORMAT_Z16, _hardware.fps);
}

void RS2Camera::_post_start_this_camera(rs2::pipeline_profile& profile) {
    rs2::device dev = profile.get_device();
    // First pause the recorder, if there is one.
    if (_uses_recorder) {
        rs2::recorder recorder = dev.as<rs2::recorder>();
        if (!recorder) {
            _log_error("_post_start_this_camera: uses_recorder but no rs2::recorder");
        } else {
            recorder.pause();
        }
    }
    RS2BaseCamera::_post_start_this_camera(profile);
}


void RS2Camera::post_start_all_cameras() {
    // First pause the recorder, if there is one.
    if (_uses_recorder) {
        rs2::pipeline_profile profile = _camera_pipeline.get_active_profile();
        rs2::device dev = profile.get_device();
        rs2::recorder recorder = dev.as<rs2::recorder>();
        if (!recorder) {
            _log_error("post_start_all_cameras: uses_recorder but no rs2::recorder");
        } else {
            recorder.resume();
        }
    } 
}
