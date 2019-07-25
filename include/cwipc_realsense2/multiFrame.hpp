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
public:
	MFCamera(std::string _serial, std::string _usb="0");
	~MFCamera();

	bool is_usb3() { return usb[0] == '3'; }
	void start(MFConfigCapture& configuration);
	std::string serial;
	std::string usb;
	rs2::pipeline pipe;
	double minx;
	double minz;
	double maxz;
};

class CWIPC_DLL_ENTRY MFCapture {

public:
	// methods
	MFCapture(const char *_configFilename=NULL);
	~MFCapture();
	cwipc_pcl_pointcloud get_pointcloud(uint64_t *timestamp); // API function that returns the merged pointcloud and timestamp
	cwipc_pcl_pointcloud get_mostRecentPointCloud();                     // return the merged cloud most recently captured/merged (don't grab a new one)
	MFConfigCamera* get_camera_config(std::string serial);
	MFCamera* get_camera(std::string serial);

	// variables
    MFConfigCapture configuration;
	uint64_t starttime;

private:
	std::string configFilename;
	// methods
	void camera_action(int camera_index, uint64_t *timestamp);// get new frames and update the camera's pointcloud
	cwipc_pcl_pointcloud merge_views();                       // merge all camera's pointclouds into one
	cwipc_pcl_pointcloud generate_pcl();                      // generate a mathematical pointcloud

	// variables
	cwipc_pcl_pointcloud mergedPC;                            // Merged pointcloud
	cwipc_pcl_pointcloud generatedPC;                         // Mathematical pointcloud for use without camera
	std::vector<MFCamera> cameras;                // Staorage of camera specifics


	// for an explanation of filtering see librealsense/doc/post-processing-filters.md and code in librealsense/src/proc 
	rs2::decimation_filter dec_filter;                        // Decimation - reduces depth frame density
	rs2::disparity_transform depth_to_disparity = rs2::disparity_transform(true);
	rs2::spatial_filter spat_filter;                          // Spatial    - edge-preserving spatial smoothing
	rs2::temporal_filter temp_filter;                         // Temporal   - reduces temporal noise
	rs2::disparity_transform disparity_to_depth = rs2::disparity_transform(false);
};
#endif /* cwipw_realsense_multiFrame_hpp */
