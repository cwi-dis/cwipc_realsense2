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

RS2PlaybackCamera::RS2PlaybackCamera(rs2::context& ctx, RS2CaptureConfig& configuration, int _camera_index, RS2CameraConfig& _camData)
:   RS2Camera(ctx, configuration, _camera_index, _camData)
{

}

RS2PlaybackCamera::~RS2PlaybackCamera() {
}

void RS2PlaybackCamera::_enable_camera(rs2::config &cfg) {
    cfg.enable_device_from_file(playback_filename);
}
