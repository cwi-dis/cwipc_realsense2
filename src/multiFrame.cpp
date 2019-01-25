//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//

#include <chrono>
#include <cstdint>

#include "multiFrame.hpp"
#include "utils.h"

//
// Stop-gap by Jack. The normal production settings are not possible over USB2.
// we should dynamically detect this (and I think we can, with the realsense
// API), but for now we use a define to lower the quality to something
// doable.
//
#define WITH_USB2
#ifndef WITH_USB2
// For USB3 devices we get the maximum possible quality and rate
const int color_width = 1280;
const int color_height = 720;
const int color_fps = 30;
const int depth_width = 1280;
const int depth_height = 720;
const int depth_fps = 30;
#else
// For USB2 we make do with lower quality and framerate
const int color_width = 640;
const int color_height = 480;
const int color_fps = 30;
const int depth_width = 640;
const int depth_height = 480;
const int depth_fps = 30;
#endif // WITH_USB2

#if defined(WIN32) || defined(_WIN32)
#define CWI_DLL_EXPORT __declspec(dllexport)
#else
#define CWI_DLL_EXPORT 
#endif

// Configure and initialize caputuring of one camera
void multiFrame::camera_start(cameradata camera_data)
{
	cout << "starting camera ser no: " << camera_data.serial << '\n';

	rs2::config cfg;
	cfg.enable_device(camera_data.serial);
	cfg.enable_stream(RS2_STREAM_COLOR, color_width, color_height, RS2_FORMAT_RGB8, color_fps);
	cfg.enable_stream(RS2_STREAM_DEPTH, depth_width, depth_height, RS2_FORMAT_Z16, depth_fps);

	camera_data.pipe.start(cfg);		// Start streaming with the configuration just set
}

// get new frames from the camera and update the pointcloud of the camera's data 
void multiFrame::camera_action(cameradata camera_data)
{
	if (!do_capture)
		return;

	rs2::pointcloud pc;
	rs2::points points;

#ifdef POLLING
	// Poll to find if there is a next set of frames from the camera
	rs2::frameset frames;
	if (!camera_data.pipe.poll_for_frames(&frames))
		return;
#else
	// Wait to find if there is a next set of frames from the camera
	auto frames = camera_data.pipe.wait_for_frames();
#endif

	auto depth = frames.get_depth_frame();
	auto color = frames.get_color_frame();
	float minz = 100.0f, maxz, minx;

	camera_data.cloud->clear();

	// Tell points frame to map to this color frame
	pc.map_to(color); // NB: This does not align the frames. That should be handled by setting resolution of cameras
	points = pc.calculate(depth);

	// Generate new vertices and color vector
	auto vertices = points.get_vertices();

	unsigned char *colors = (unsigned char*)color.get_data();

	// Set the background removal window
	for (int i = 0; i < points.size(); i++) {
		if (vertices[i].z != 0 && minz > vertices[i].z) {
			minz = vertices[i].z;
			minx = vertices[i].x;
		}
	}
	maxz = 0.8f + minz;

	// Make PointCloud
	for (int i = 0; i < points.size(); i++) {
		float x = minx - vertices[i].x; x *= x;
		float z = vertices[i].z;
		if (minz < z && z < maxz - x) { // Simple background removal, horizontally parabolic, vertically straight.
			PointT pt;
			pt.x = vertices[i].x;
			pt.y = -vertices[i].y;
			pt.z = -z;
			int pi = i * 3;
			pt.r = colors[pi];
			pt.g = colors[pi + 1];
			pt.b = colors[pi + 2];
			if (!green_screen || noChromaRemoval(&pt)) // chromakey removal
				camera_data.cloud->push_back(pt);
		}
	}
}

void multiFrame::merge_views(boost::shared_ptr<PointCloudT> cloud_ptr)
{
	PointCloudT *aligned_cld = new PointCloudT();

	cloud_ptr->clear();
	for (cameradata ccfg : CameraData) {
		PointCloudT *cam_cld = ccfg.cloud.get();
		if (cam_cld->size() > 0) {
			transformPointCloud(*cam_cld, *aligned_cld, *ccfg.trafo);
			for (PointT pnt : *aligned_cld)
				cloud_ptr->push_back(pnt);
		}
	}
	aligned_cld->clear();
	delete aligned_cld;
}

// API function that triggers the capture and returns the merged pointcloud and timestamp
void multiFrame::get_pointcloud(uint64_t *timestamp, void **pointcloud)
{
	*timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

	if (CameraData.size() > 0) {
		for (cameradata ccfg : CameraData)
			camera_action(ccfg);
		merge_views(RingBuffer[ring_index]);
		if (RingBuffer[ring_index].get()->size() > 0) {
#ifdef DEBUG
			cout << "capturer produced a merged cloud of " << RingBuffer[ring_index].get()->size() << " points in ringbuffer " << ring_index << "\n";
#endif
			*pointcloud = reinterpret_cast<void *> (&(RingBuffer[ring_index]));
		}
		else {
#ifdef DEBUG
			cout << "\nWARNING: capturer did get an empty pointcloud\n\n";
#endif
			// HACK to make sure the encoder does not get an empty pointcloud 
			PointT point;
			point.x = 1.0;
			point.y = 1.0;
			point.z = 1.0;
			point.rgb = 0.0;
			RingBuffer[ring_index]->points.push_back(point);
			*pointcloud = reinterpret_cast<void *> (&(RingBuffer[ring_index]));
		}
	}
	else {	// return a spinning generated mathematical pointcloud
		angle += 0.031415;
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()));
		transformPointCloud(*GeneratedPC, *RingBuffer[ring_index], transform);
		*pointcloud = reinterpret_cast<void *> (&(RingBuffer[ring_index]));
	}
	ring_index = ring_index < ringbuffer_size - 1 ? ++ring_index : 0;
}


///////////////////////////
// class captureIt stuff //
///////////////////////////


void captureIt::getPointCloud(uint64_t *timestamp, void **pointcloud) {
	static multiFrame mFrame;

#ifdef DEBUG
	cout << "captureIt is asked for a pointcloud\n";
#endif

	mFrame.get_pointcloud(timestamp, pointcloud);

#ifdef DEBUG
	boost::shared_ptr<PointCloudT> captured_pc;
	captured_pc = *reinterpret_cast<boost::shared_ptr<PointCloudT>*>(*pointcloud);
	cout << "captureIt handed over a pointcloud of " << captured_pc.get()->size() << " points\n";
	cout.flush();
#endif
}

extern "C" void CWI_DLL_EXPORT getPointCloud(uint64_t *timestamp, void **pointcloud) {
	captureIt captureit;
	captureit.getPointCloud(timestamp, pointcloud);
}
