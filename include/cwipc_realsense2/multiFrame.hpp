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
	rs2::pipeline pipe;
	boost::shared_ptr<Eigen::Affine3d> trafo;
	cwipc_pcl_pointcloud cloud;
};

struct configdata {
	bool background_removal = true;		// If true reduces pointcloud to forground object 
	bool greenscreen_removal = true;	// If true include greenscreen removal
	bool tiling = false;				// If true produce tiled stream
	double cloud_resolution = 0.0;		// Resolution of voxelized pointclouds
	double tile_resolution = 0.01;		// Resolution of tiling process
	unsigned int ringbuffer_size = 1;	// Size of the ringbuffer
	std::vector<cameradata> camera_data;// Storage of per camera data
};

class CWIPC_DLL_ENTRY multiFrame {

public:
	multiFrame();
	~multiFrame();

	// API function that returns the merged pointcloud and timestamp
	cwipc_pcl_pointcloud get_pointcloud(uint64_t *timestamp);

	// return the merged cloud 
	cwipc_pcl_pointcloud getPointCloud();


	configdata Configuration;

private:

	// Configure and initialize caputuring of one camera
	void camera_start(cameradata* camera_data);

	// get new frames from the camera and update the pointcloud of the camera's data 
	void camera_action(cameradata camera_data);

	void merge_views(cwipc_pcl_pointcloud cloud_ptr);

	// generate a mathematical pointcloud
	cwipc_pcl_pointcloud generate_pcl();

	// Globals
	std::vector<cwipc_pcl_pointcloud> RingBuffer;	// Buffer of merged pointclouds
	cwipc_pcl_pointcloud GeneratedPC;				// Mathematical pointcloud for use without camera

	int ring_index = 0;								// counter for ring buffer
};
#endif /* cwipw_realsense_multiFrame_hpp */
