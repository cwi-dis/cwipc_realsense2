#ifndef cwipc_realsense_RS2PlaybackCapture_hpp
#define cwipc_realsense_RS2PlaybackCapture_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

#include <librealsense2/rs.hpp>

#include "RS2Config.hpp"
#include "RS2BaseCapture.hpp"
#include "RS2PlaybackCamera.hpp"

class RS2PlaybackCapture : public RS2BaseCapture {
private:
    RS2PlaybackCapture();   
public:
    virtual ~RS2PlaybackCapture();
    static int count_devices() { return 0; }
    static RS2PlaybackCapture* factory() { return new RS2PlaybackCapture(); }
    bool eof() override {return false; }
    bool seek(uint64_t timestamp) override; 
protected:
    virtual bool _create_cameras() override;
    virtual void _setup_camera_sync() override {};
    virtual void _setup_camera_hardware_parameters() override {};
    virtual bool _check_cameras_connected() override { return true;};
    virtual bool _apply_config(const char* configFilename) override;
    virtual void _initial_camera_synchronization() override;
private:
    std::string base_directory = "";
    uint64_t earliest_recording_timestamp_seen = 0;
};
#endif // cwipc_realsense_RS2PlaybackCapture_hpp
