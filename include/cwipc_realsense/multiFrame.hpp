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

#include "tinyxml.h"
#include "defs.h"

#undef DEBUG
//#define DEBUG
#undef POLLING
//#define POLLING

using namespace std::chrono;

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
		GeneratedPC = generate_pcl();

		// prepare storage for camera data for each connected camera
		for (auto dev : devs) {
			if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name) {
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
		if (CameraData.size() == 0)		// no camera connected, so we'll use a generated pointcloud instead
			std::cout << "No cameras found, a spinning generated pointcloud of " << GeneratedPC->size() << " data points will be offered instead\n" ;

		file2config();		// set the configuratioon (transformation matrices, ringbuffersize, greenscreen option, etc.)

		for (int i = 0; i < ringbuffer_size; i++) {
			boost::shared_ptr<PointCloudT> buf(new PointCloudT());
			RingBuffer.push_back(buf);
		}

		do_capture = true;
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
		return RingBuffer[ring_index];
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

	// return the serialnumber of the specified camera
	string getCameraSerial(int i)
	{
		if (i >= 0 && i < CameraData.size())
			return CameraData[i].serial;
		else
			return string("0");
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
		file->SetAttribute("ringbuffersize", ringbuffer_size);
		file->SetAttribute("greenscreenremoval", green_screen);

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
			std::cout << "WARNING: Failed to load cameraconfig.xml\n";
			if (CameraData.size() > 1)	
				std::cout << "\tCaptured pointclouds will be merged based on unregistered camera clouds\n";
			return;
		}

		TiXmlHandle docHandle(&doc);
		TiXmlElement* configElement = docHandle.FirstChild("file").FirstChild("CameraConfig").ToElement();
		configElement->QueryDoubleAttribute("resolution", &spatial_resolution);
		configElement->QueryUnsignedAttribute("ringbuffersize", &ringbuffer_size);
		ringbuffer_size = ringbuffer_size < 1 ? 1 : ringbuffer_size;
		configElement->QueryBoolAttribute("greenscreenremoval", &green_screen);

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

	// generate a mathematical pointcloud
	PointCloudT::Ptr generate_pcl()
	{
		PointCloudT::Ptr point_cloud_ptr(new PointCloudT);
		uint8_t r(255), g(15), b(15);
		for (float z(-1.0f); z <= 1.0f; z += 0.005f) {
			for (float angle(0.0); angle <= 360.0; angle += 1.0f) {
				PointT point;
				point.x = 0.5f*cosf(deg2rad(angle))*(1.0f - z*z);
				point.y = sinf(deg2rad(angle))*(1.0f - z*z);
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

	// Globals
	vector<cameradata> CameraData;						// Storage of per camera data
	vector<boost::shared_ptr<PointCloudT>> RingBuffer;	// Buffer of merged pointclouds
	boost::shared_ptr<PointCloudT> GeneratedPC;			// Mathematical pointcloud for use without camera
	float angle = 0.0f;									// Rotation of generated PC
	bool do_capture = false;								// Switch for "pause"
	double spatial_resolution = 0.0;						// Resolution of voxelized pointclouds
	int ring_index = 0;									// counter for ring buffer
	unsigned int ringbuffer_size = 1;					// Size of the ringbuffer
	bool green_screen = true;							// If true include greenscreen removal
};


///////////////////////////
// class captureIt stuff //
///////////////////////////

class captureIt
{
public:
	void getPointCloud(uint64_t *timestamp, void **pointcloud);
};

#endif /* cwipw_realsense_multiFrame_hpp */
