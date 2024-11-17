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
private:
    RS2PlaybackCapture();   
public:
    ~RS2PlaybackCapture();
    static int count_devices() { return 0; }
    static RS2PlaybackCapture* factory() { return new RS2PlaybackCapture(); }
    
protected:
    virtual void _setup_camera_sync() override {};
    virtual void _setup_camera_hardware_parameters() override {};
    virtual bool _check_cameras_connected() override { return true; /* xxxjack could check filenames... */};
    virtual bool _apply_config(const char* configFilename) override;
    virtual bool _create_cameras() override;
    virtual void _initial_camera_synchronization() override;
    std::string base_directory = "";
};
#endif // cwipc_realsense_RS2PlaybackCapture_hpp
