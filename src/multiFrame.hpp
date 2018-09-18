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

#include "tinyxml.h"

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

struct cameradata {
	const char* serial;
	Eigen::Affine3d* trafo;
	boost::shared_ptr<PointCloud<PointXYZRGB>> cloud;
};

class multiFrame {

public:
	multiFrame() {
		rs2::context ctx;    // Create librealsense context for managing devices
		auto devs = ctx.query_devices();
		const std::string platform_camera_name = "Platform Camera";
		MergedCloud = generate_pcl();

		for (auto dev : devs) {
			if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name)
				// Start capturing for all connected RealSense devices
				capture_start(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), this);
		}

		if (CameraData.size() == 0) {
			// no cameras connected, then generate a pointcloud instead
			RotatedPC = generate_pcl();
			std::cout << "No cameras found, a spinning generated pointcloud of " << MergedCloud->size() << " data points will be offered" << std::endl;
		}

		// set the transformation matrices to align all cameras
		file2config();
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

	// store the current camera transformation setting into a xml ducument
	void config2file()
	{
		std::lock_guard<std::mutex> guard(frames_mutex);

		TiXmlDocument doc;
		doc.LinkEndChild(new TiXmlDeclaration("1.0", "", ""));

		TiXmlElement* root = new TiXmlElement("file");
		doc.LinkEndChild(root);

		TiXmlElement* file = new TiXmlElement("CameraConfig");
		root->LinkEndChild(file);

		for (cameradata ccfg : CameraData) {
			TiXmlElement* cam = new TiXmlElement("camera");
			cam->SetAttribute("serial", ccfg.serial);
			file->LinkEndChild(cam);

			TiXmlElement* trafo = new TiXmlElement("trafo");
			cam->LinkEndChild(trafo);

			TiXmlElement* val = new TiXmlElement("values");
			val->SetDoubleAttribute("v00", (*ccfg.trafo)(0, 0));
			val->SetDoubleAttribute("v01", (*ccfg.trafo)(0, 1));
			val->SetDoubleAttribute("v02", (*ccfg.trafo)(0, 2));
			val->SetDoubleAttribute("v03", (*ccfg.trafo)(0, 3));
			val->SetDoubleAttribute("v10", (*ccfg.trafo)(1, 0));
			val->SetDoubleAttribute("v11", (*ccfg.trafo)(1, 1));
			val->SetDoubleAttribute("v12", (*ccfg.trafo)(1, 2));
			val->SetDoubleAttribute("v13", (*ccfg.trafo)(1, 3));
			val->SetDoubleAttribute("v20", (*ccfg.trafo)(2, 0));
			val->SetDoubleAttribute("v21", (*ccfg.trafo)(2, 1));
			val->SetDoubleAttribute("v22", (*ccfg.trafo)(2, 2));
			val->SetDoubleAttribute("v23", (*ccfg.trafo)(2, 3));
			val->SetDoubleAttribute("v30", (*ccfg.trafo)(3, 0));
			val->SetDoubleAttribute("v31", (*ccfg.trafo)(3, 1));
			val->SetDoubleAttribute("v32", (*ccfg.trafo)(3, 2));
			val->SetDoubleAttribute("v33", (*ccfg.trafo)(3, 3));

			trafo->LinkEndChild(val);
		}

		doc.SaveFile("cameraconfig.xml");
	}

	// restore the camera transformation setting as stored in the configuration document
	void file2config()
	{
		TiXmlDocument doc("cameraconfig.xml");
		bool loadOkay = doc.LoadFile();
		if (!loadOkay)
		{
			std::cout << "Failed to load cameraconfig.xml \n";
			return;
		}
		std::lock_guard<std::mutex> guard(frames_mutex);

		TiXmlHandle docHandle(&doc);
		TiXmlElement* camConfig = docHandle.FirstChild("file").FirstChild("CameraConfig").FirstChild("camera").ToElement();

		while (camConfig)
		{
			const char * serial = camConfig->Attribute("serial");
			for (cameradata ccfg : CameraData) {
				if (strcmp(ccfg.serial, serial) == 0) {
					TiXmlElement *trafo = camConfig->FirstChildElement("trafo");
					if (trafo) {
						TiXmlElement *val = trafo->FirstChildElement("values");
						val->QueryDoubleAttribute("v00", &((*ccfg.trafo)(0, 0)));
						val->QueryDoubleAttribute("v01", &((*ccfg.trafo)(0, 1)));
						val->QueryDoubleAttribute("v02", &((*ccfg.trafo)(0, 2)));
						val->QueryDoubleAttribute("v03", &((*ccfg.trafo)(0, 3)));
						val->QueryDoubleAttribute("v10", &((*ccfg.trafo)(1, 0)));
						val->QueryDoubleAttribute("v11", &((*ccfg.trafo)(1, 1)));
						val->QueryDoubleAttribute("v12", &((*ccfg.trafo)(1, 2)));
						val->QueryDoubleAttribute("v13", &((*ccfg.trafo)(1, 3)));
						val->QueryDoubleAttribute("v20", &((*ccfg.trafo)(2, 0)));
						val->QueryDoubleAttribute("v21", &((*ccfg.trafo)(2, 1)));
						val->QueryDoubleAttribute("v22", &((*ccfg.trafo)(2, 2)));
						val->QueryDoubleAttribute("v23", &((*ccfg.trafo)(2, 3)));
						val->QueryDoubleAttribute("v30", &((*ccfg.trafo)(3, 0)));
						val->QueryDoubleAttribute("v31", &((*ccfg.trafo)(3, 1)));
						val->QueryDoubleAttribute("v32", &((*ccfg.trafo)(3, 2)));
						val->QueryDoubleAttribute("v33", &((*ccfg.trafo)(3, 3)));
					}
				}
			}
			camConfig = camConfig->NextSiblingElement("camera");
		}
	}

	Eigen::Affine3d *getCameraTransform(const char* serial)
	{
		for (cameradata ccfg : CameraData) {
			if (strcmp(ccfg.serial, serial) == 0)
				return ccfg.trafo;
		}
		return NULL;
	}

	boost::shared_ptr<PointCloud<PointXYZRGB>> getCameraCloud(const char * serial)
	{
		for (cameradata ccfg : CameraData) {
			if (strcmp(ccfg.serial, serial) == 0)
				return ccfg.cloud;
		}
		return NULL;
	}

	std::mutex frames_mutex;
	void capture_start(const std::string& serial_number, multiFrame* m_frame);
	boost::shared_ptr<PointCloud<PointXYZRGB>> merge_views();
	boost::shared_ptr<PointCloud<PointXYZRGB>> MergedCloud;
	boost::shared_ptr<PointCloud<PointXYZRGB>> RotatedPC;

	// Storage of the per camera data
	std::vector<cameradata> CameraData;
	float angle = 0;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
