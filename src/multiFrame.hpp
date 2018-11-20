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
#include <functional>
#include <fstream>
#include <ctime>
#include <chrono>
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

typedef PointXYZRGB PointT;
typedef PointCloud<PointT> PointCloudT;

struct cameradata {
	string serial;
	rs2::pipeline pipe;
	boost::shared_ptr<Eigen::Affine3d> trafo;
	boost::shared_ptr<PointCloudT> cloud;
};

class multiFrame {

public:
	multiFrame() {
		// Create librealsense context for managing all connected RealSense devices
		rs2::context ctx;
		auto devs = ctx.query_devices();

		const std::string platform_camera_name = "Platform Camera";
		MergedCloud = generate_pcl();
		GeneratedPC = generate_pcl();

		for (auto dev : devs) {
			if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name) {
				// prepare storage for camera data
				boost::shared_ptr<PointCloudT> empty_pntcld(new PointCloudT());
				boost::shared_ptr<Eigen::Affine3d> default_trafo(new Eigen::Affine3d());
				default_trafo->setIdentity();
				cameradata* cc = new cameradata();
				cc->serial = string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
				cc->cloud = empty_pntcld;
				cc->trafo = default_trafo;
				CameraData.push_back(*cc);
				camera_start(*cc);
			}
		}
		do_capture = true;

		if (CameraData.size() > 1) {
			// set the transformation matrices to align all cameras
			file2config();
		}
		if (CameraData.size() == 0) {
			// no cameras connected, then generate a pointcloud instead
			RotatedPC = generate_pcl();
			std::cout << "No cameras found, a spinning generated pointcloud of " << RotatedPC->size() << " data points will be offered" << std::endl;
		}
	}

	~multiFrame() {
		for (cameradata ccfg : CameraData) {
			ccfg.pipe.stop();
		}
		cout << "stopped all camera's\n";
	}

	// API function that returns the merged pointcloud and timestamp
	void get_pointcloud(uint64_t *timestamp, void **pointcloud);


	// return the merged cloud 
	boost::shared_ptr<PointCloudT> getPointCloud()
	{
		if (CameraData.size() > 0)
			return MergedCloud;
		else
			return RotatedPC; 
	}

	// return the number of connected and recognized cameras
	int getNumberOfCameras() {
		return CameraData.size();
	}

	// return the cloud captured by the specified camera
	boost::shared_ptr<PointCloudT> getCameraCloud(int i)
	{
		if (i >= 0 && i < CameraData.size())
			return CameraData[i].cloud;
		else
			return NULL;
	}

	// return the transformation matrix of the specified camera
	string getCameraSerial(int i)
	{
		if (i >= 0 && i < CameraData.size())
			return CameraData[i].serial;
		else
			return NULL;
	}

	// return the transformation matrix of the specified camera
	Eigen::Affine3d *getCameraTransform(int i)
	{
		if (i >= 0 && i < CameraData.size())
			return CameraData[i].trafo.get();
		else
			return NULL;
	}

	void capture_start()
	{
		if (do_capture)
			return;
		do_capture = true;
	}

	void pauze_capture()
	{
		if (!do_capture)
			return;
		do_capture = false;
	}

	// store the current camera transformation setting into a xml ducument
	void config2file()
	{
		TiXmlDocument doc;
		doc.LinkEndChild(new TiXmlDeclaration("1.0", "", ""));

		TiXmlElement* root = new TiXmlElement("file");
		doc.LinkEndChild(root);

		TiXmlElement* file = new TiXmlElement("CameraConfig");
		root->LinkEndChild(file);
		file->SetDoubleAttribute("resolution", spatial_resolution);

		for (cameradata ccfg : CameraData) {
			TiXmlElement* cam = new TiXmlElement("camera");
			cam->SetAttribute("serial", ccfg.serial.c_str());
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

private:

	// restore the camera transformation setting as stored in the configuration document
	void file2config()
	{
		TiXmlDocument doc("cameraconfig.xml");
		bool loadOkay = doc.LoadFile();
		if (!loadOkay)
		{
			std::cout << "WARNING: Failed to load cameraconfig.xml\n\tPointcloud will be a merger of unregistrated camera clouds\n";
			return;
		}

		TiXmlHandle docHandle(&doc);
		TiXmlElement* configElement = docHandle.FirstChild("file").FirstChild("CameraConfig").ToElement();
		configElement->QueryDoubleAttribute("resolution", &spatial_resolution);

		TiXmlElement* cameraElement = configElement->FirstChildElement("camera");
		while (cameraElement)
		{
			const char * serial = cameraElement->Attribute("serial");
			for (cameradata ccfg : CameraData) {
				if (ccfg.serial == serial) {
					TiXmlElement *trafo = cameraElement->FirstChildElement("trafo");
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
			cameraElement = cameraElement->NextSiblingElement("camera");
		}
	}

	// return the transformation matrix of the specified camera
	Eigen::Affine3d *getCameraTransform(const char* serial)
	{
		for (cameradata ccfg : CameraData) {
			if (ccfg.serial == serial)
				return ccfg.trafo.get();
		}
		return NULL;
	}

	PointCloudT* copyCloud(PointCloudT pcl)
	{
		PointCloudT *copiedCloud = new PointCloudT(pcl);
		return copiedCloud;
	}

	// generate a mathematical pointcloud
	PointCloudT::Ptr generate_pcl()
	{
		PointCloudT::Ptr point_cloud_ptr(new PointCloudT);
		uint8_t r(255), g(15), b(15);
		for (float z(-1.0); z <= 1.0; z += 0.005) {
			for (float angle(0.0); angle <= 360.0; angle += 1.0) {
				PointT point;
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

	cameradata* getCameraData(const char * serial)
	{
		for (cameradata ccfg : CameraData) {
			if (ccfg.serial == serial)
				return &ccfg;
		}
		return NULL;
	}

	// Methods
	void camera_start(cameradata camera_data);
	void camera_action(cameradata camera_data);
	void merge_views(boost::shared_ptr<PointCloudT> pcl);

	// Storage of per camera data
	vector<cameradata> CameraData;

	// Globals
	boost::shared_ptr<PointCloudT> MergedCloud;
	boost::shared_ptr<PointCloudT> GeneratedPC;
	boost::shared_ptr<PointCloudT> RotatedPC;
	bool do_capture = false;	// switch for "pause"
	float angle = 0.0f;			// rotation of generated PC
	double spatial_resolution = 0.0;
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
