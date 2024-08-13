#ifndef cwipc_realsense_RS2Capture_hpp
#define cwipc_realsense_RS2Capture_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

#include <librealsense2/rs.hpp>

#include <cwipc_util/internal.h>

#include "cwipc_realsense2/private/RS2Config.hpp"

class RS2Camera;

class RS2Capture : public CwipcBaseCapture {
protected:
    // methods
    RS2Capture();
    virtual bool _apply_config(const char* configFilename);
    bool _apply_default_config();
    void _find_camera_positions();
    virtual void _setup_camera_sync();
    virtual void _setup_camera_hardware_parameters();
    virtual void _refresh_camera_hardware_parameters();
    virtual bool _check_cameras_connected();
    void _unload_cameras();

public:
    static int count_devices();
    static RS2Capture* factory() { return new RS2Capture(); }
    virtual ~RS2Capture();
    virtual bool config_reload(const char *configFilename);
    std::string config_get();
    cwipc* get_pointcloud(); // API function that returns the merged pointcloud
    bool pointcloud_available(bool wait);                     // Returns true if a pointcloud is available
    cwipc* get_mostRecentPointCloud();                     // return the merged cloud most recently captured/merged (don't grab a new one)
    RS2CameraConfig* get_camera_config(std::string serial);
    RS2Camera* get_camera(std::string serial);
    float get_pointSize();
    bool map2d3d(int tile, int x_2d, int y_2d, int d_2d, float* out3d);

    // variables
    RS2CaptureConfig configuration;
    uint64_t starttime = 0;
    int numberOfPCsProduced = 0;
    int camera_count = 0; // number of cameras
                          //
    void request_image_auxdata(bool _rgb, bool _depth) {
        want_auxdata_rgb = _rgb;
        want_auxdata_depth = _depth;
    }

    bool want_auxdata_rgb = false;
    bool want_auxdata_depth = false;

protected:
    static rs2::context* ctx_p;             // librealsense2 context (coordinates all cameras)
                                            //
    static inline rs2::context& ctx() { 
        if (ctx_p == nullptr) {
            ctx_p = new rs2::context();
        }
        return *ctx_p;
    }

    virtual bool _create_cameras();
    std::vector<RS2Camera*> cameras;                // Storage of camera specifics
    void _control_thread_main();              // Internal: main thread that controls per-camera grabbing and processing and combines pointclouds.
    bool stopped = false;
    std::thread *control_thread = nullptr;

private:
    void merge_views();                       // Internal: merge all camera's pointclouds into one
    void _request_new_pointcloud();           // Internal: request a new pointcloud to be grabbed and processed
    cwipc* mergedPC = nullptr;                            // Merged pointcloud
    std::mutex mergedPC_mutex;                                // Lock for all mergedPC-related dta structures
    bool mergedPC_is_fresh = false;                                   // True if mergedPC contains a freshly-created pointcloud
    std::condition_variable mergedPC_is_fresh_cv;             // Condition variable for signalling freshly-created pointcloud
    bool mergedPC_want_new = false;                                   // Set to true to request a new pointcloud
    std::condition_variable mergedPC_want_new_cv;             // Condition variable for signalling we want a new pointcloud
};

#endif // cwipc_realsense_RS2Capture_hpp
