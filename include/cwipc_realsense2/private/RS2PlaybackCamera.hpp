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
    RS2PlaybackCamera(rs2::context& ctx, RS2CaptureConfig& configuration, int _camera_index, RS2CameraConfig& _camData, std::string recording_filename);
    ~RS2PlaybackCamera();

    virtual void post_start_all_cameras() override;
protected:
    virtual void _pre_start(rs2::config &cfg) override;
    virtual void _post_start(rs2::pipeline_profile& profile) override;
private:
    std::string playback_filename;

};
#endif // cwipc_realsense_RS2PlaybackCamera_hpp
