#ifndef cwipc_realsense_RS2Capture_hpp
#define cwipc_realsense_RS2Capture_hpp
#pragma once

#include "RS2BaseCapture.hpp"

class RS2Capture : public RS2BaseCapture {
public:
    using RS2BaseCapture::RS2BaseCapture;
    virtual ~RS2Capture();
    static int count_devices();
    static RS2Capture* factory();

    bool eof() override;
    bool seek(uint64_t timestamp) override; 
protected:
    RS2Capture();
    virtual bool _create_cameras() override;
    virtual void _setup_camera_sync() override;
    virtual void _setup_camera_hardware_parameters() override;
    virtual bool _check_cameras_connected() override;
    virtual bool _apply_config(const char* configFilename) override;
    virtual void _initial_camera_synchronization() override;
private:
    /// Helper for _apply_config: setup default configuration for all connected cameras.
    bool _apply_auto_config();
};
#endif // cwipc_realsense_RS2Capture_hpp
