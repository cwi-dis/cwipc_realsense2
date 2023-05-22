#ifndef cwipc_realsense_RS2Capture_hpp
#define cwipc_realsense_RS2Capture_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

#include <librealsense2/rs.hpp>

#include "cwipc_realsense2/private/RS2Config.hpp"

class RS2Camera;

class RS2Capture {
protected:
	RS2Capture(int dummy);
	int _count_devices();
	bool _apply_config(const char* configFilename);
	bool _apply_default_config();
	void _find_camera_positions();
	void _setup_camera_sync();
	void _setup_camera_hardware_parameters();
    void _unload_cameras();
public:
	// methods
	RS2Capture(const char *configFilename=NULL);
	virtual ~RS2Capture();
    bool config_reload(const char *configFilename);
    std::string config_get();
	cwipc* get_pointcloud(); // API function that returns the merged pointcloud
	bool pointcloud_available(bool wait);					  // Returns true if a pointcloud is available
	cwipc* get_mostRecentPointCloud();                     // return the merged cloud most recently captured/merged (don't grab a new one)
	RS2CameraConfig& get_camera_data(std::string serial);
	RS2Camera* get_camera(std::string serial);
	float get_pointSize();

	// variables
    RS2CaptureConfig configuration;
	uint64_t starttime;
	int numberOfPCsProduced;
	int camera_count; // number of cameras
    void request_image_auxdata(bool _rgb, bool _depth) {
        want_auxdata_rgb = _rgb;
        want_auxdata_depth = _depth;
    }
    bool want_auxdata_rgb;
    bool want_auxdata_depth;
protected:
	rs2::context ctx;				// librealsense2 context (coordinates all cameras)
	std::list<rs2::device> rs2_cameras; // librealsense2 cameras that we use
	virtual void _create_cameras();
	std::vector<RS2Camera*> cameras;                // Storage of camera specifics
	void _control_thread_main();              // Internal: main thread that controls per-camera grabbing and processing and combines pointclouds.
	bool stopped;
	std::thread *control_thread;

private:
	void merge_views();                       // Internal: merge all camera's pointclouds into one
	void _request_new_pointcloud();           // Internal: request a new pointcloud to be grabbed and processed
	cwipc* mergedPC;                            // Merged pointcloud
	std::mutex mergedPC_mutex;                                // Lock for all mergedPC-related dta structures
	bool mergedPC_is_fresh;                                   // True if mergedPC contains a freshly-created pointcloud
	std::condition_variable mergedPC_is_fresh_cv;             // Condition variable for signalling freshly-created pointcloud
	bool mergedPC_want_new;                                   // Set to true to request a new pointcloud
	std::condition_variable mergedPC_want_new_cv;             // Condition variable for signalling we want a new pointcloud
};
#endif // cwipc_realsense_RS2Capture_hpp
