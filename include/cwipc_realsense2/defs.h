//
//  defs.h
//
//  Created by Fons Kuijk on 12-12-18.
//

#ifndef cwipc_realsense2_defs_h
#define cwipc_realsense2_defs_h

#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/voxel_grid.h>

// Define to enable optional dumping of RGB video frames (to test hardware sync)
#define WITH_DUMP_VIDEO_FRAMES

//
// Definitions of types used across cwipc_realsense2, cwipc_codec and cwipc_util.
//
#include "cwipc_util/api_pcl.h"
#include "offlinedefs.h"

struct MFCameraSettings {
	bool do_decimation = false;
	int decimation_value = 1;             // int value between 2 and 8
	bool do_threshold = true;
	double threshold_near = 0.15;         // float, near point for distance threshold
	double threshold_far = 6.0;           // float, far point for distance threshold
	bool do_spatial = true;
	int spatial_iterations = 2;           // int val between 1 and 5
	double spatial_alpha = 0.5;          // val between 0.25 and 1.0
	int spatial_delta = 30;               // int val between 1 and 50
	int spatial_filling = 0;	          // int val between 0 and 6
	bool do_temporal = false;
	double temporal_alpha = 0.4;	      // val between 0 and 1
	int temporal_delta = 20;	          // val between 1 and 100
	int temporal_percistency = 3;         // val between 0 and 8
};

struct MFCameraData {
	std::string serial;		// Serial number of this camera
	boost::shared_ptr<Eigen::Affine3d> trafo;	//!< Transformation matrix from camera coorindates to world coordinates
	boost::shared_ptr<Eigen::Affine3d> intrinsicTrafo;	//!< offline only: matrix to convert color to depth coordinates
	cwipc_vector cameraposition;	//!< Position of this camera in real world coordinates
	cwipc_vector background;
	cwipc_pcl_pointcloud cloud;	//!< Pointcloud most recently captured
};

struct MFCaptureConfig {
	// system data
	int usb3_width = 1280;
	int usb3_height = 720;
	int usb3_fps = 30;
	int usb2_width = 640;
	int usb2_height = 480;
	int usb2_fps = 15;

	// processing data
	bool background_removal = false;      // If true reduces pointcloud to forground object
	bool greenscreen_removal = false;	  // If true include greenscreen removal
	bool depth_filtering = false;         // If true perform post filtering on depth frame
	double height_min = 0.0;			  // If height_min != height_max perform height filtering
	double height_max = 0.0;			  // If height_min != height_max perform height filtering
	double cloud_resolution = 0.0;        // Resolution of voxelized pointclouds
	bool density = false;			  	  // Grab with high density (alternative is high accuracy)
	bool tiling = false;	              // If true produce tiled stream
	double tiling_resolution = 0.01;      // Resolution of tiling process
	std::string tiling_method = "";       // Method of tiling process

	// special features
	std::string cwi_special_feature = ""; // Specifier for temporary development specific feature

	MFCameraSettings default_camera_settings;
	// realsense specific post processing filtering

	// per camera data
	std::vector<MFCameraData> cameraData;
};
#endif /* cwipc_realsense2_defs_h */
