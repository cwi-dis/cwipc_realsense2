#ifndef cwipc_realsense_RS2PlaybackCapture_hpp
#define cwipc_realsense_RS2PlaybackCapture_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

#include <librealsense2/rs.hpp>

#include "cwipc_realsense2/private/RS2Config.hpp"
#include "cwipc_realsense2/private/RS2Capture.hpp"
#include "cwipc_realsense2/private/RS2PlaybackCamera.hpp"

class RS2PlaybackCapture : public RS2Capture {
public:
    // methods
    RS2PlaybackCapture();
    ~RS2PlaybackCapture();
    virtual bool config_reload(const char* configFilename) override;
    static int count_devices() { return 0; }
    static RS2PlaybackCapture* factory() { return new RS2PlaybackCapture(); }
};
#endif // cwipc_realsense_RS2PlaybackCapture_hpp
