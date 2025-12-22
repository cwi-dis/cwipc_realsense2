#ifndef cwipc_realsense_RS2BaseCapture_hpp
#define cwipc_realsense_RS2BaseCapture_hpp
#pragma once


#include <thread>
#include <mutex>
#include <condition_variable>

#include <librealsense2/rs.hpp>

#include "cwipc_util/capturers.hpp"

#include "RS2Config.hpp"

class RS2Camera;

/** Base class for capturers that use the librealsense API. 
 * 
 * For librealsense actually most of the implementation is in this class,
 * because a playback device has the same API as a live camera.
 * 
 * Subclasses need to implement factory() and count_devices().
*/
class RS2BaseCapture : public CwipcBaseCapture {
public:
    /// Subclasses need to implement static factory().
    /// Subclasses need to implement static count_devices().

    using CwipcBaseCapture::CwipcBaseCapture;
    virtual ~RS2BaseCapture();

    int get_camera_count() override final;
    bool is_valid() override final;
    virtual bool config_reload(const char *configFilename) override final;
    std::string config_get() override final;

    //
    // This section has the public capturer-independent API used during normal runtime.
    //

    /// Returns true when a new point cloud is available.
    bool pointcloud_available(bool wait) override final;
    /// Returns the new point cloud. The caller is now the owner of this point cloud.
    cwipc* get_pointcloud() override final;
    /// Returns a reasonable point size for the current capturer.
    float get_pointSize() override final;
    /// Return 3D point for a given camera, given RGB image 2D coordinates.
    bool map2d3d(int tile, int x_2d, int y_2d, int d_2d, float* out3d) override final;
    /// Return 2D point in depth image coordinates given 2D point in color image coordinates.
    bool mapcolordepth(int tile, int u, int v, int* out2d) override final;
    virtual bool eof() override = 0;
    /// Seek to given timestamp (only implemented for playback capturers).
    virtual bool seek(uint64_t timestamp) override = 0;
protected:
    RS2BaseCapture();
public:
     /// Tell the capturer that each point cloud should also include RGB and/or D images and/or RGB/D capture timestamps.
    void request_auxdata(bool _rgb, bool _depth, bool _timestamps);
   
protected:
    /// Methods that are different for live vs playback capturers..
    /// Create the per-camera capturers.
    virtual bool _create_cameras() = 0;
    /// Setup camera synchronization (if needed).
    virtual void _setup_camera_sync() = 0;
    /// Setup camera hardware parameters (white balance, etc).
    virtual void _setup_camera_hardware_parameters() = 0;
    /// Check that all cameras are connected.
    virtual bool _check_cameras_connected() = 0;
    
protected:
    /// Methods that are not different for live vs playback capturers..
    /// Get hardware parameters into our configuration structure.
    virtual void _refresh_camera_hardware_parameters();
    /// Unload all cameras and release all resources.
    void _unload_cameras();

    RS2CameraConfig* get_camera_config(std::string serial);
    void _control_thread_main();
    virtual void _initial_camera_synchronization();
    void _find_camera_positions();
    /// Load configuration from file or string.
    virtual bool _apply_config(const char* configFilename);
    /// Load default configuration based on hardware cameras connected.
    bool _apply_auto_config();


    void merge_camera_pointclouds();
    void _request_new_pointcloud();
    
public:
    /// Current configuration. Has to be public because cwipc_realsense2 needs access to all sorts of internals
    /// of it, but it would be better if this access was readonly...
    RS2CaptureConfig configuration;
    
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

#endif