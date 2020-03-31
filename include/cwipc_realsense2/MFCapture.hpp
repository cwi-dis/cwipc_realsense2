#ifndef cwipc_realsense_MFCapture_hpp
#define cwipc_realsense_MFCapture_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

#include <librealsense2/rs.hpp>

#include "defs.h"

#ifndef CWIPC_DLL_ENTRY
#if defined(WIN32) || defined(_WIN32)
#define CWIPC_DLL_ENTRY __declspec(dllimport)
#else
#define CWIPC_DLL_ENTRY 
#endif
#endif

class MFCamera;

class CWIPC_DLL_ENTRY MFCapture {
protected:
	MFCapture(int dummy);
public:
	// methods
	MFCapture(const char *configFilename=NULL);
	virtual ~MFCapture();
	cwipc_pcl_pointcloud get_pointcloud(uint64_t *timestamp); // API function that returns the merged pointcloud and timestamp
	bool pointcloud_available(bool wait);					  // Returns true if a pointcloud is available
	cwipc_pcl_pointcloud get_mostRecentPointCloud();                     // return the merged cloud most recently captured/merged (don't grab a new one)
	MFCameraData& get_camera_data(std::string serial);
	MFCamera* get_camera(std::string serial);
	float get_pointSize();

	// variables
    MFCaptureConfig configuration;
	uint64_t starttime;
	int numberOfPCsProduced;
    bool no_cameras;                        // True of no cameras attached
protected:
	rs2::context ctx;				// librealsense2 context (coordinates all cameras)
	virtual void _create_cameras(rs2::device_list devs);
	std::vector<MFCamera*> cameras;                // Storage of camera specifics
	void _control_thread_main();              // Internal: main thread that controls per-camera grabbing and processing and combines pointclouds.
	bool stopped;
	std::thread *control_thread;

private:
	void merge_views();                       // Internal: merge all camera's pointclouds into one
	void _request_new_pointcloud();           // Internal: request a new pointcloud to be grabbed and processed
	cwipc_pcl_pointcloud mergedPC;                            // Merged pointcloud
	std::mutex mergedPC_mutex;                                // Lock for all mergedPC-related dta structures
	bool mergedPC_is_fresh;                                   // True if mergedPC contains a freshly-created pointcloud
	std::condition_variable mergedPC_is_fresh_cv;             // Condition variable for signalling freshly-created pointcloud
	bool mergedPC_want_new;                                   // Set to true to request a new pointcloud
	std::condition_variable mergedPC_want_new_cv;             // Condition variable for signalling we want a new pointcloud
};
#endif // cwipc_realsense_MFCapture_hpp
