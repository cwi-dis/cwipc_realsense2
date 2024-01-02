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

#include "cwipc_realsense2/private/RS2PlaybackCamera.hpp"

RS2PlaybackCamera::RS2PlaybackCamera(rs2::context& ctx, RS2CaptureConfig& configuration, int _camera_index, RS2CameraConfig& _camData, std::string _filename)
:   RS2Camera(ctx, configuration, _camera_index, _camData),
    playback_filename(_filename)
{

}

RS2PlaybackCamera::~RS2PlaybackCamera() {
}

void RS2PlaybackCamera::_pre_start(rs2::config &cfg) {
    cfg.enable_device_from_file(playback_filename);
}

void RS2PlaybackCamera::_post_start() {
#if 0
    rs2::device_list devices_after = context.query_devices();
    std::cerr << "xxxjack post_start device count " << devices_after.size() << std::endl;
#endif
    rs2::pipeline_profile prof = pipe.get_active_profile();
    rs2::device dev = prof.get_device();
    std::cerr << "xxxjack dev info RS2_CAMERA_INFO_NAME " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
    std::cerr << "xxxjack dev info RS2_CAMERA_INFO_SERIAL_NUMBER " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
}