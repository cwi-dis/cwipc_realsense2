//
//  multiFrame.hpp
//
//  Created by Fons Kuijk on 23-04-18.
//

#ifndef multiFrame_hpp
#define multiFrame_hpp
#pragma once

#include <atomic>
#include <stdio.h>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <thread>
#include <ctime>
#include <chrono>
#include <mutex>
#include <algorithm>

#include <librealsense2/rs.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
/*
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/time.h> /**/

using namespace std;
using namespace pcl;
using namespace std::chrono;

class multiFrame {

public:
	multiFrame() {
		rs2::context ctx;    // Create librealsense context for managing devices
		auto devs = ctx.query_devices();
		PointCloud_views.resize(4);
		const std::string platform_camera_name = "Platform Camera";

		numberOfCameras = 0;
		for (auto dev : devs) {
			if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name)
				// Start capturing for all connected RealSense devices
				capture_start(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), this, numberOfCameras++);
		}
		PointCloud_views.resize(numberOfCameras);
		if (numberOfCameras == 0) {
			// no cameras connected, then generate a pointcloud instead
			merged_cloud = generate_pcl();
			rot_pc = generate_pcl();
			std::cout << "No cameras found, a spinning generated pointcloud of " << merged_cloud->size() << " data points will be used instead" << std::endl;
		}
	}

	void get_pointcloud(uint64_t *timestamp, void **pointcloud);

private:
	std::mutex frames_mutex;
	void capture_start(const std::string& serial_number, multiFrame* m_frame, int camNum);
	void push_pointcloud(boost::shared_ptr<PointCloud<PointXYZRGB>> pntcld, const int camNum);
	boost::shared_ptr<PointCloud<PointXYZRGB>> merge_views();
	PointCloud<PointXYZRGB>::Ptr generate_pcl();

	std::vector<boost::shared_ptr<PointCloud<PointXYZRGB>>> PointCloud_views;
	boost::shared_ptr<PointCloud<PointXYZRGB>> merged_cloud;
	boost::shared_ptr<PointCloud<PointXYZRGB>> rot_pc;;
	int numberOfCameras = 0;
	float angle = 0;
};

class captureIt
{
public:
	void getPointCloud(uint64_t *timestamp, void **pointcloud);
};

#endif /* multiFrame_hpp */
