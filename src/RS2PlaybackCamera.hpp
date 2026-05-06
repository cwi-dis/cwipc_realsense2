#ifndef cwipc_realsense_RS2PlaybackCamera_hpp
#define cwipc_realsense_RS2PlaybackCamera_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>

#include "RS2Config.hpp"
#include "RS2Camera.hpp"

class RS2PlaybackCamera : public RS2BaseCamera {
public:
    RS2PlaybackCamera(rs2::context& ctx, RS2CaptureConfig& configuration, RS2CaptureMetadataConfig& metadata, int _camera_index, std::string recording_filename);
    ~RS2PlaybackCamera();

    virtual void post_start_all_cameras() override final;
    virtual bool seek(uint64_t timestamp) override final;
    virtual bool eof() override final { return _playback_eof; }
    virtual void pause();
    virtual void resume();
protected:
    virtual void _init_pipeline_for_this_camera(rs2::config &cfg) override;
    virtual void _post_start_this_camera(rs2::pipeline_profile& profile) override;
private:
    uint64_t _previous_color_timestamp = 0;
    std::string _playback_filename;
    bool _playback_realtime = false;
    bool _playback_loop = true;
    bool _playback_eof = false;
};
#endif // cwipc_realsense_RS2PlaybackCamera_hpp
