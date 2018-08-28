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
#include <Eigen/StdVector>

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
		const std::string platform_camera_name = "Platform Camera";
		MergedCloud = generate_pcl();

		numberOfCameras = 0;
		for (auto dev : devs) {
			if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name)
				// Start capturing for all connected RealSense devices
				capture_start(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), this, numberOfCameras++);
		}

		if (numberOfCameras == 0) {
			// no cameras connected, then generate a pointcloud instead
			RotatedPC = generate_pcl();
			std::cout << "No cameras found, a spinning generated pointcloud of " << MergedCloud->size() << " data points will be offered" << std::endl;
		}
	}

	void get_pointcloud(uint64_t *timestamp, void **pointcloud);

private:

	// generate a mathematical pointcloud
	PointCloud<PointXYZRGB>::Ptr generate_pcl()
	{
		PointCloud<PointXYZRGB>::Ptr point_cloud_ptr(new PointCloud<PointXYZRGB>);
		uint8_t r(255), g(15), b(15);
		for (float z(-1.0); z <= 1.0; z += 0.005) {
			for (float angle(0.0); angle <= 360.0; angle += 1.0) {
				PointXYZRGB point;
				point.x = 0.5*cosf(deg2rad(angle))*(1 - z*z);
				point.y = sinf(deg2rad(angle))*(1 - z*z);
				point.z = z;
				uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
				point.rgb = *reinterpret_cast<float*>(&rgb);
				point_cloud_ptr->points.push_back(point);
			}
			if (z < 0.0) { r -= 1; g += 1; }
			else { g -= 1; b += 1; }
		}
		point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
		point_cloud_ptr->height = 1;
		return point_cloud_ptr;
	}

	std::mutex frames_mutex;
	void capture_start(const std::string& serial_number, multiFrame* m_frame, int camNum);
	void push_pointcloud(boost::shared_ptr<PointCloud<PointXYZRGB>> pntcld, const int camNum);
	boost::shared_ptr<PointCloud<PointXYZRGB>> merge_views();
	boost::shared_ptr<PointCloud<PointXYZRGB>> MergedCloud;
	boost::shared_ptr<PointCloud<PointXYZRGB>> RotatedPC;

	std::vector<boost::shared_ptr<PointCloud<PointXYZRGB>>> PointCloudViews;
	std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Vector4f>> MergeTrafos;

	int numberOfCameras = 0;
	float angle = 0;
};


///////////////////////////
// class captureIt stuff //
///////////////////////////

class captureIt
{
public:
	void getPointCloud(uint64_t *timestamp, void **pointcloud);
};

#endif /* multiFrame_hpp */
