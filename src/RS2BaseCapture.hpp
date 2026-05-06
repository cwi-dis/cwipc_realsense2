#ifndef cwipc_realsense_RS2BaseCapture_hpp
#define cwipc_realsense_RS2BaseCapture_hpp
#pragma once

#include "cwipc_util/internal/capturers.hpp"
#include "RS2BaseCamera.hpp"
#include "RS2Config.hpp"

#include <librealsense2/rs.hpp>

#include <condition_variable>
#include <iostream>
#include <fstream>
#include <mutex>
#include <thread>


/** Base class for capturers that use the librealsense API. 
 * 
 * For librealsense actually most of the implementation is in this class,
 * because a playback device has the same API as a live camera.
 * 
 * Subclasses need to implement factory() and count_devices().
*/
class RS2BaseCapture : public CwipcBaseCapture {
public:
    using CwipcBaseCapture::CwipcBaseCapture;
    virtual ~RS2BaseCapture();

    //
    // Base class interface implementation
    //

    virtual int get_camera_count() override final;
    virtual bool can_start() override final;
    virtual bool is_playing() override final;
    virtual bool config_reload(const char* configFilename) override final;
    virtual bool start() override final;
    virtual void stop() override final;
    virtual std::string config_get() const override final;
    virtual CwipcBaseCameraConfig const* get_camera_config(size_t index) const;
    virtual void request_metadata(bool rgb, bool depth, bool timestamps, bool skeleton) override final;

    bool pointcloud_available(bool wait) override final;
    cwipc_pointcloud* get_pointcloud() override final;
    float get_pointSize() override final;
    virtual bool map2d3d(int tile, int x_2d, int y_2d, int d_2d, float* out3d) override final;
    virtual bool mapcolordepth(int tile, int u, int v, int* out2d) override final;
    virtual bool eof() override final;

    //
    // The seek method will be implemented by the child classes to fully realize the class
    //

    /// Seek to given timestamp (only implemented for playback capturers).
    virtual bool seek(uint64_t timestamp) override = 0;
   
protected:
    //
    // Protected implementation shared across derived classes.
    //

    /// Load configuration from file or string.
    /// Note: not final, because some subclasses need to extend it.
    virtual bool _apply_config(const char* configFilename) override;
    /// Get configuration for a single camera, by serial number.
    RS2CameraConfig const* _get_camera_config(std::string serial) const;
    /// Sets the connected state of a camera
    void _set_camera_connected(const std::string& serial, bool connected);
    /// xxxjack another one?
    virtual void _initial_camera_synchronization() override;
    /// Start all cameras.
    virtual bool _start_cameras() override final;
    /// Stop and unload all cameras and release all resources.
    virtual void _unload_cameras() override final;
    /// Stop all cameras.
    virtual void _stop_cameras() override final;
    /// Create the cameraconfig file for the recording, if needed.
    virtual void _post_stop_all_cameras() override final;
    /// Control thread main
    void _control_thread_main();

    /// Anything that needs to be done to get the camera streams synchronized after opening.
    /// (Realsense Playback seeks all streams to the same timecode, the earliest one present
    /// in each stream)

    virtual bool _capture_all_cameras(uint64_t& timestamp) final;
    void _request_new_pointcloud();
    void _merge_camera_pointclouds();

    //
    // Abstract protected interface - will be implemented by derived classes.
    //

    /// Load default configuration based on hardware cameras connected.
    virtual bool _apply_auto_config() override = 0;
    /// Setup camera synchronization (if needed).
    virtual bool _setup_inter_camera_sync() override = 0;
    /// Create the per-camera capturers.
    virtual bool _create_cameras() override = 0;
    /// Setup camera hardware parameters (white balance, etc).
    virtual bool _init_hardware_for_all_cameras() override = 0;
    /// Check that all cameras are connected.
    virtual bool _check_cameras_connected() override = 0;

protected:
    RS2CaptureConfig configuration;                 ///< Current configuration.
    RS2CaptureMetadataConfig metadata;              ///< The meta-data configuration.
    
    rs2::context capturer_context;

    std::vector<RS2BaseCamera*> cameras;            ///< The per-camera capturers
    bool _is_initialized = false;
    bool stopping = false;
    bool stopped = false;
    bool _eof = false;

    cwipc_pointcloud* mergedPC = nullptr;           ///< Merged pointcloud
    std::mutex mergedPC_mutex;                      ///< Lock for all mergedPC-related dta structures
    
    bool mergedPC_is_fresh = false;                 ///< True if mergedPC contains a freshly-created pointcloud
    std::condition_variable mergedPC_is_fresh_cv;   ///< Condition variable for signalling freshly-created pointcloud
    
    bool mergedPC_want_new = false;                 ///< Set to true to request a new pointcloud
    std::condition_variable mergedPC_want_new_cv;   ///< Condition variable for signalling we want a new pointcloud

    std::thread *control_thread = nullptr;
    
};

#endif