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

struct RS2CameraConfig {
	bool do_decimation = false;
	int decimation_value = 1;             // int value between 2 and 8
	bool do_threshold = true;
	double threshold_near = 0.15;         // float, near point for distance threshold
	double threshold_far = 6.0;           // float, far point for distance threshold
    int depth_x_erosion = 0;              // How many valid depth pixels to remove in camera x direction
    int depth_y_erosion = 0;              // How many valid depth pixels to remove in camera y direction
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

struct RS2CameraData {
	std::string serial;		// Serial number of this camera
    std::string type = "realsense";       // Camera type (must be realsense)
	pcl::shared_ptr<Eigen::Affine3d> trafo;	//!< Transformation matrix from camera coorindates to world coordinates
	pcl::shared_ptr<Eigen::Affine3d> intrinsicTrafo;	//!< offline only: matrix to convert color to depth coordinates
	cwipc_vector cameraposition;	//!< Position of this camera in real world coordinates
	cwipc_pcl_pointcloud cloud;	//!< Pointcloud most recently captured
};

struct RS2CaptureConfig {
	// system data
	int usb3_width = 1280;
	int usb3_height = 720;
	int usb3_fps = 30;
	int usb2_width = 640;
	int usb2_height = 480;
	int usb2_fps = 15;
    bool usb2allowed = false;                    // true if USB2 is allowed

	// processing data
	bool greenscreen_removal = false;	  // If true include greenscreen removal
	double height_min = 0.0;			  // If height_min != height_max perform height filtering
	double height_max = 0.0;			  // If height_min != height_max perform height filtering
	bool density = false;			  	  // Grab with high density (alternative is high accuracy)
    int exposure = -1;                    // Set exposure for both color and depth. -1 is auto.
    int whitebalance = -1;                // Set whitebalance for color camera. -1 is auto.
    int backlight_compensation = -1;      // Set backlight compesnation for color camera. -1 is don't change.
    int laser_power = 360;                // Laser power. -1 is don't change.

	// special features
    bool want_auxdata_rgb = false;
    bool want_auxdata_depth = false;
    
	RS2CameraConfig camera_config;
	// realsense specific post processing filtering

	// per camera data
	std::vector<RS2CameraData> camera_data;
};

#endif /* cwipc_realsense2_defs_h */
