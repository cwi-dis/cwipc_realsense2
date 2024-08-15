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
    // Constructor is protected (use factory function)
    RS2Capture();
public:
    virtual ~RS2Capture();
    
    /// Return the number of RealSense cameras attached to this computer.
    static int count_devices();
    /// Create a new RS2Capture multi-camera capturer.
    static RS2Capture* factory() { return new RS2Capture(); }

    /// Reload configuration, possibly restarting capturer and cameras.
    virtual bool config_reload(const char *configFilename);
    /// Get complete current configuration as JSON string.
    std::string config_get();

    /// Returns true when a new point cloud is available.
    bool pointcloud_available(bool wait);
    /// Returns the new point cloud. The caller is now the owner of this point cloud.
    cwipc* get_pointcloud();
    /// Returns a reasonable point size for the current capturer.
    float get_pointSize();
    /// Tell the capturer that each point cloud should also include RGB and/or D images.
    void request_image_auxdata(bool _rgb, bool _depth);
    /// Return 3D point for a given camera, given RGB image 2D coordinates.
    bool map2d3d(int tile, int x_2d, int y_2d, int d_2d, float* out3d);

protected:
    virtual bool _create_cameras();
    void _unload_cameras();

    RS2CameraConfig* get_camera_config(std::string serial);
    void _control_thread_main();
    void _find_camera_positions();
    virtual bool _apply_config(const char* configFilename);
    bool _apply_default_config();

    virtual void _setup_camera_sync();
    virtual void _setup_camera_hardware_parameters();
    virtual void _refresh_camera_hardware_parameters();
    virtual bool _check_cameras_connected();

    void merge_camera_pointclouds();
    void _request_new_pointcloud();
    
public:
    /// Current configuration. Has to be public because cwipc_realsense2 needs access to all sorts of internals
    /// of it, but it would be better if this access was readonly...
    RS2CaptureConfig configuration;
    /// This is the number of active, working, cameras.
    int camera_count = 0;

protected:
    rs2::context capturer_context;

    std::vector<RS2Camera*> cameras;    ///< The per-camera capturers
    bool stopped = false;
    std::thread *control_thread = nullptr;
    cwipc* mergedPC = nullptr;          ///< Merged pointcloud
    std::mutex mergedPC_mutex;          ///< Lock for all mergedPC-related dta structures
    bool mergedPC_is_fresh = false;     ///< True if mergedPC contains a freshly-created pointcloud
    std::condition_variable mergedPC_is_fresh_cv;   ///< Condition variable for signalling freshly-created pointcloud
    bool mergedPC_want_new = false;     ///< Set to true to request a new pointcloud
    std::condition_variable mergedPC_want_new_cv;   ///< Condition variable for signalling we want a new pointcloud
};

#endif // cwipc_realsense_RS2Capture_hpp
