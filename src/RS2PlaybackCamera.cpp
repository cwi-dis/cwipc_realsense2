//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//
#include <cstdlib>

// Define to get (a little) debug prints
#define CWIPC_DEBUG
#undef CWIPC_DEBUG_THREAD
#define CWIPC_DEBUG_SYNC

// This is the dll source, so define external symbols as dllexport on windows.

#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif

#include "cwipc_realsense2/private/RS2PlaybackCamera.hpp"

RS2PlaybackCamera::RS2PlaybackCamera(rs2::context& ctx, RS2CaptureConfig& configuration, int _camera_index, std::string _filename)
:   RS2Camera(ctx, configuration, _camera_index),
    playback_filename(_filename),
    playback_realtime(configuration.playback_realtime)
{

}

RS2PlaybackCamera::~RS2PlaybackCamera() {
}

void RS2PlaybackCamera::_pre_start(rs2::config &cfg) {
    cfg.enable_device_from_file(playback_filename);
}

void RS2PlaybackCamera::_post_start(rs2::pipeline_profile& profile) {
    rs2::device dev = profile.get_device();
    rs2::playback playback = dev.as<rs2::playback>();

    playback.pause();
    playback.set_real_time(playback_realtime);
    
    // Seek device, if needed
    if (camera_config.playback_inpoint_micros != 0) {
        uint64_t new_pos = ((uint64_t)1000) * camera_config.playback_inpoint_micros;
        uint64_t old_pos = playback.get_position();
#ifdef CWIPC_DEBUG
        if (debug) std::cerr << "RS2PlaybackCamera::_post_start: pos was " << old_pos << " seek to " << new_pos << std::endl;
#endif
        playback.seek(std::chrono::nanoseconds(new_pos));
    }
    RS2Camera::_post_start(profile);
}

void RS2PlaybackCamera::post_start_all_cameras() {
    rs2::pipeline_profile prof = camera_pipeline.get_active_profile();
    rs2::device dev = prof.get_device();
    rs2::playback playback = dev.as<rs2::playback>();
    playback.resume();
    if (debug) std::cerr << "RS2PlaybackCamera::_post_start_all_cameras: playback resumed at " << playback.get_position() << std::endl;
}
