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
private:
    RS2PlaybackCamera(const RS2PlaybackCamera&);  // Disable copy constructor
    RS2PlaybackCamera& operator=(const RS2PlaybackCamera&);   // Disable assignment
public:
    RS2PlaybackCamera(rs2::context& ctx, RS2CaptureConfig& configuration, int _camera_index, RS2CameraConfig& _camData, std::string recording_filename);
    ~RS2PlaybackCamera();
protected:
    virtual void _pre_start(rs2::config &cfg) override;
    virtual void _post_start() override;
private:
    std::string playback_filename;
#ifdef xxxjack_old
    void _start_capture_thread();
    void _capture_thread_main();
private:
    int depth_width, depth_height, depth_bpp, depth_fps;
    rs2_format depth_format;
    int color_width, color_height, color_bpp, color_fps;
    rs2_format color_format;
    rs2_extrinsics depth_to_color_extrinsics;
    rs2::frameset current_frameset;
    rs2::software_device dev;
    rs2::software_sensor depth_sensor;
    rs2::software_sensor color_sensor;
    rs2::stream_profile color_stream;
    rs2::stream_profile depth_stream;
    rs2::syncer sync;
    int feedFrameNum;
#endif
};
#endif // cwipc_realsense_RS2PlaybackCamera_hpp
