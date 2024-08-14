#ifndef cwipc_realsense_RS2Camera_hpp
#define cwipc_realsense_RS2Camera_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

#include <librealsense2/rs.hpp>

#include <cwipc_util/internal.h>

#include "cwipc_realsense2/private/RS2Config.hpp"

class RS2Camera : CwipcBaseCamera {
public:
    RS2Camera(rs2::context& _ctx, RS2CaptureConfig& configuration, int _camera_index);
    virtual ~RS2Camera();

    /// First step in starting: starts the camera. Called for all cameras. 
    void start_camera();
    /// Second step in starting: starts the capturer. Called after all cameras have been started.
    virtual void start_capturer();
    /// Third step in starting, called after all capturers have been started.
    virtual void post_start_all_cameras() {}
    /// Completely stops camera and capturer, releases all resources. Can be re-started with start_camera, etc.
    void stop_camera_and_capturer();

    /// Step 1 in capturing: wait for a valid frameset. Any image processing will have been done. 
    bool wait_for_captured_frameset();
    /// Step 2: Forward the frameset to the processing thread to turn it into a point cloud.
    void create_pc_from_frameset();
    /// Step 2a: Save auxdata from frameset into given cwipc object.
    void save_frameset_auxdata(cwipc *pc);
    /// Step 2b: get timestamp from frameset.
    uint64_t get_frameset_timestamp();
    /// Step 3: Wait for the point cloud processing.
    void wait_for_pc_created();
    /// Step 3a: borrow a pointer to the point cloud just created, as a PCL point cloud.
    cwipc_pcl_pointcloud access_current_pcl_pointcloud() { return current_pointcloud; }
    
    /// Implementation of mapping 2D color image coordinates to 3D coordinates.
    bool map2d3d(int x_2d, int y_2d, int d_2d, float* out3d);
    /// Get camera hardware parameters, or check that they match what we got from another camera.
    bool getHardwareParameters(RS2CameraHardwareConfig& output, bool match);

protected:
    virtual void _pre_start(rs2::config& cfg);
    virtual void _post_start(rs2::pipeline_profile& profile);

    void _init_filters();

    void _start_processing_thread();
    void _processing_thread_main();
    virtual void _start_capture_thread();
    virtual void _capture_thread_main();
    
    void _erode_depth(rs2::depth_frame, int x_delta, int y_delta);

    void _init_current_pointcloud(int size);
    void _computePointSize(rs2::pipeline_profile profile);
    void transformPoint(cwipc_pcl_point& out, const rs2::vertex& in);
    void transformPoint(float *out, const float *in);
    

public:
    float pointSize; ///< Needed by RS2Capture, computed once at camera start

    int camera_index; ///< Needed by RS2Capture
    std::string serial; ///< Needed by RS2Capture

protected:
    RS2CameraConfig& camera_config;
    RS2CaptureProcessingConfig& processing;
    RS2CameraProcessingParameters& filtering;
    RS2CameraHardwareConfig& hardware;
    RS2CaptureAuxdataConfig& auxData;
    std::string record_to_file;
    
    bool camera_stopped;
    bool camera_pipeline_started;

    std::thread *camera_processing_thread;
    std::thread *camera_capture_thread;
    
    rs2::context capturer_context;
    rs2::pipeline camera_pipeline;
    
    rs2::frame_queue captured_frame_queue;
    rs2::frame_queue processing_frame_queue;
    std::mutex processing_mutex;
    std::condition_variable processing_done_cv;
    bool processing_done;
    
    rs2::frameset current_captured_frameset;
    rs2::frameset current_processed_frameset;
    cwipc_pcl_pointcloud current_pointcloud;

    // for an explanation of filtering see librealsense/doc/post-processing-filters.md and code in librealsense/src/proc
    rs2::align align_color_to_depth;                 // Align depth and color data
    rs2::align align_depth_to_color;                   // Align depth and color data
    rs2::decimation_filter dec_filter;                        // Decimation - reduces depth frame density
    // rs2::hdr_merge not supported yet.
    // rs2::sequence_id_filter not supported yet
    rs2::threshold_filter threshold_filter;                   // Thresholding: minimum and maximum distance
    rs2::disparity_transform depth_to_disparity = rs2::disparity_transform(true);
    rs2::spatial_filter spatial_filter;                          // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temporal_filter;                         // Temporal   - reduces temporal noise
    rs2::hole_filling_filter hole_filling_filter;
    rs2::disparity_transform disparity_to_depth = rs2::disparity_transform(false);
    rs2::pointcloud depth_to_pointcloud;     // The pointcloud constructor

};

#endif // cwipc_realsense_RS2Camera_hpp
