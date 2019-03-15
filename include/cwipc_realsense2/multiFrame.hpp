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

#undef CWIPC_DEBUG
//#define CWIPC_DEBUG
#undef POLLING
//#define POLLING

#ifndef CWIPC_DLL_ENTRY
#if defined(WIN32) || defined(_WIN32)
#define CWIPC_DLL_ENTRY __declspec(dllimport)
#else
#define CWIPC_DLL_ENTRY 
#endif
#endif

using namespace std::chrono;

struct cameradata {
	std::string serial;
	std::string usb;
	rs2::pipeline pipe;
	boost::shared_ptr<Eigen::Affine3d> trafo;
	double background_x = 0.0;
	double background_y = 0.0;
	double background_z = 0.0;
	double minx;
	double minz;
	double maxz;
	cwipc_pcl_pointcloud cloud;
};

struct configdata {
	// system data
	int usb3_width = 1280;
	int usb3_height = 720;
	int usb3_fps = 30;
	int usb2_width = 640;
	int usb2_height = 480;
	int usb2_fps = 15;

	// processing data
	bool background_removal = true;		// If true reduces pointcloud to forground object
	bool greenscreen_removal = true;		// If true include greenscreen removal
	bool depth_filtering = false;		// If true perform post filtering on depth frame
	double cloud_resolution = 0.0;		// Resolution of voxelized pointclouds
	bool tiling = false;					// If true produce tiled stream
	double tile_resolution = 0.01;		// Resolution of tiling process

	// per camera data
	std::vector<cameradata> camera_data;
};


class CWIPC_DLL_ENTRY multiFrame {

public:
	// methods
	multiFrame();
	~multiFrame();
	cwipc_pcl_pointcloud get_pointcloud(uint64_t *timestamp);	// API function that returns the merged pointcloud and timestamp
	cwipc_pcl_pointcloud getPointCloud();				// return the merged cloud
	
	// variables
    configdata configuration;

private:
	// methods
	void camera_start(cameradata* camera_data);			// Configure and initialize caputuring of one camera
	void camera_action(cameradata* camera_data);			// get new frames and update the camera's pointcloud
	cwipc_pcl_pointcloud merge_views();					// merge all camera's pointclouds into one
	cwipc_pcl_pointcloud generate_pcl();					// generate a mathematical pointcloud

	// variables
	cwipc_pcl_pointcloud MergedPC ;						// Merged pointcloud
	cwipc_pcl_pointcloud GeneratedPC;					// Mathematical pointcloud for use without camera
	rs2::decimation_filter dec_filter;					// Decimation - reduces depth frame density
	rs2::disparity_transform depth_to_disparity = rs2::disparity_transform(true);
	rs2::spatial_filter spat_filter;						// Spatial    - edge-preserving spatial smoothing
	rs2::temporal_filter temp_filter;					// Temporal   - reduces temporal noise
	rs2::disparity_transform disparity_to_depth = rs2::disparity_transform(false);
};
#endif /* cwipw_realsense_multiFrame_hpp */
