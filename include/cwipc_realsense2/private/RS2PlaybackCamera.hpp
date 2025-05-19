#ifndef cwipc_realsense_RS2PlaybackCamera_hpp
#define cwipc_realsense_RS2PlaybackCamera_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>

#include "cwipc_realsense2/private/RS2Config.hpp"
#include "cwipc_realsense2/private/RS2Camera.hpp"

class RS2PlaybackCamera : public RS2Camera {
public:
    RS2PlaybackCamera(rs2::context& ctx, RS2CaptureConfig& configuration, int _camera_index, std::string recording_filename);
    ~RS2PlaybackCamera();

    virtual void post_start_all_cameras() override;
    virtual bool seek(uint64_t timestamp) override;
    virtual void pause();
    virtual void resume();
protected:
    virtual void _pre_start(rs2::config &cfg) override;
    virtual void _post_start(rs2::pipeline_profile& profile) override;
#if 0
    virtual rs2::frameset wait_for_frames() override;
#endif
    uint64_t previous_color_timestamp = 0;
private:
    std::string playback_filename;
    bool playback_realtime = false;
    bool playback_loop = true;
};
#endif // cwipc_realsense_RS2PlaybackCamera_hpp
