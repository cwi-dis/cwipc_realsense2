//
//  multiFrame.hpp
//
//  Created by Fons Kuijk on 23-04-18
//

#ifndef cwipw_realsense_multiFrame_hpp
#define cwipw_realsense_multiFrame_hpp
#pragma once

#include <atomic>
#include <stdio.h>
#include <vector>
#include <map>
#include <iostream>
#include <functional>
#include <fstream>
#include <ctime>
#include <chrono>
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <librealsense2/rs.hpp>
#include <Eigen/StdVector>

#include "defs.h"

#ifndef CWIPC_DLL_ENTRY
#if defined(WIN32) || defined(_WIN32)
#define CWIPC_DLL_ENTRY __declspec(dllimport)
#else
#define CWIPC_DLL_ENTRY 
#endif
#endif

using namespace std::chrono;

class MFCamera {
private:
	MFCamera(const MFCamera&);	// Disable copy constructor
	MFCamera& operator=(const MFCamera&);	// Disable assignment
public:
	MFCamera(rs2::context& ctx, MFCaptureConfig& configuration, int _camera_index, MFCameraData& _camData, std::string _usb="0");
	~MFCamera();

	bool is_usb3() { return usb[0] == '3'; }
	void start(MFCaptureConfig& configuration);
	void start_capturer();
	void stop();
	void capture_frameset();
	void create_pc_from_frames();
	void wait_for_pc();
	void dump_color_frame(const std::string& filename);
public:
	// This is public because MFCapture needs it when dumping the color images
	rs2::frameset current_frameset;
public:
	// These are public because pcl_align wants to access them
	double minx;
	double minz;
	double maxz;
	int camera_index;
	std::string serial;

private:
	MFCameraData& camData;
	std::string usb;
	rs2::pipeline pipe;
	bool do_depth_filtering;
	bool do_background_removal;
	bool do_greenscreen_removal;
	bool stopped;
	std::thread *grabber_thread;
	std::thread *processing_thread;
	rs2::frame_queue captured_frame_queue;
	rs2::frame_queue processing_frame_queue;
	std::mutex processing_mutex;
	std::condition_variable processing_done_cv;
	bool processing_done;
	// for an explanation of filtering see librealsense/doc/post-processing-filters.md and code in librealsense/src/proc
	rs2::decimation_filter dec_filter;                        // Decimation - reduces depth frame density
	rs2::disparity_transform depth_to_disparity = rs2::disparity_transform(true);
	rs2::spatial_filter spat_filter;                          // Spatial    - edge-preserving spatial smoothing
	rs2::temporal_filter temp_filter;                         // Temporal   - reduces temporal noise
	rs2::disparity_transform disparity_to_depth = rs2::disparity_transform(false);
	void _process_depth_frame(rs2::depth_frame &depth);
	void _capture_thread_main();
	void _processing_thread_main();

};

class CWIPC_DLL_ENTRY MFCapture {

public:
	// methods
	MFCapture(const char *_configFilename=NULL);
	~MFCapture();
	cwipc_pcl_pointcloud get_pointcloud(uint64_t *timestamp); // API function that returns the merged pointcloud and timestamp
	cwipc_pcl_pointcloud get_mostRecentPointCloud();                     // return the merged cloud most recently captured/merged (don't grab a new one)
	MFCameraData& get_camera_data(std::string serial);
	MFCamera* get_camera(std::string serial);

	// variables
    MFCaptureConfig configuration;
	uint64_t starttime;

private:
	rs2::context ctx;				// librealsense2 context (coordinates all cameras)
	std::string configFilename;
	// methods
	void merge_views();                       // merge all camera's pointclouds into one
	cwipc_pcl_pointcloud generate_pcl();                      // generate a mathematical pointcloud

	// variables
	cwipc_pcl_pointcloud mergedPC;                            // Merged pointcloud
	cwipc_pcl_pointcloud generatedPC;                         // Mathematical pointcloud for use without camera
	std::vector<MFCamera*> cameras;                // Storage of camera specifics


};
#endif /* cwipw_realsense_multiFrame_hpp */
