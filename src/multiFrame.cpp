//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18.
//
#include <chrono>
#include <cstdint>
#include "multiFrame.hpp"

/*
const int color_width = 1280;
const int color_height = 720;
const int color_fps = 30;
const int depth_width = 1280;
const int depth_height = 720;
const int depth_fps = 30;
/**/
const int color_width = 640;
const int color_height = 360;
const int color_fps = 15;
const int depth_width = 640;
const int depth_height = 360;
const int depth_fps = 15;
/**/

// Configure and initialize caputuring of one camera
void multiFrame::camera_start(cameradata camera_data)
{
	std::cout << "starting camera ser no: " << camera_data.serial << '\n';

		rs2::config cfg;
		cfg.enable_device(camera_data.serial);
		cfg.enable_stream(RS2_STREAM_COLOR, color_width, color_height, RS2_FORMAT_RGB8, color_fps);
		cfg.enable_stream(RS2_STREAM_DEPTH, depth_width, depth_height, RS2_FORMAT_Z16, depth_fps);

		camera_data.pipe.start(cfg);		// Start streaming with the configuration just set
}

void multiFrame::camera_action(cameradata camera_data)
{
	if (!do_capture)
		return;

	rs2::pointcloud pc;
	rs2::points points;

	// Wait for the next set of frames from the camera
	auto frames = camera_data.pipe.wait_for_frames();
	auto depth = frames.get_depth_frame();
	auto color = frames.get_color_frame();
	float min = 100.f, max, minx;

	// Tell points frame to map to this color frame
	pc.map_to(color); // NB: This does not align the frames. That should be handled by setting resolution of cameras
	points = pc.calculate(depth);

	// Generate new vertices and color vector
	auto vertices = points.get_vertices();

	unsigned char *colors = (unsigned char*)color.get_data();

	// Find the nearest point
	for (int i = 0; i < points.size(); i++)
		if (vertices[i].z != 0 && min > vertices[i].z) {
			min = vertices[i].z;
			minx = vertices[i].x;
		}

	// Set the maximum distance
	max = 0.8f + min;

	// Make PointCloud
	camera_data.cloud->clear();

	for (int i = 0; i < points.size(); i++) {
		float x = minx - vertices[i].x; x *= x;
		float z = vertices[i].z;
		if (min < z && z < max - x) { // Simple background removal, horizontally parabolic, vertically straight.
			PointXYZRGB pt;
			pt.x = vertices[i].x;
			pt.y = -vertices[i].y;
			pt.z = -z;
			int pi = i * 3;
			pt.r = colors[pi];
			pt.g = colors[pi + 1];
			pt.b = colors[pi + 2];
			camera_data.cloud->push_back(pt);
		}
	}
}

void multiFrame::merge_views(boost::shared_ptr<PointCloud<PointXYZRGB>> cloud_ptr)
{
	PointCloud<PointXYZRGB> *aligned_cld = new PointCloud<PointXYZRGB>();

	cloud_ptr->clear();
	for (cameradata ccfg : CameraData) {
		PointCloud<PointXYZRGB> *cam_cld = ccfg.cloud.get();
		if (cam_cld->size() > 0) {
			transformPointCloud(*cam_cld, *aligned_cld, *ccfg.trafo);
			for (PointXYZRGB pnt : *aligned_cld)
				cloud_ptr->push_back(pnt);
		}
	}
	aligned_cld->clear();
	delete aligned_cld;
}

// API function that returns the merged pointcloud and timestamp
void multiFrame::get_pointcloud(uint64_t *timestamp, void **pointcloud)
{
	*timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

	if (CameraData.size() > 0) {
		for (cameradata ccfg : CameraData)
			camera_action(ccfg);
		merge_views(MergedCloud);
		if (MergedCloud.get()->size() > 0)
			*pointcloud = reinterpret_cast<void *> (&MergedCloud);
		else
			// HACK to make sure the encoder does not get an empty pointcloud //
			*pointcloud = reinterpret_cast<void *> (&GeneratedPC);
	}
	else {	// return a spinning generated mathematical pointcloud
		angle += 0.031415;
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()));
		transformPointCloud(*GeneratedPC, *RotatedPC, transform);
		*pointcloud = reinterpret_cast<void *> (&RotatedPC);
	}
}


///////////////////////////
// class captureIt stuff //
///////////////////////////

void captureIt::getPointCloud(uint64_t *timestamp, void **pointcloud) {
	static multiFrame mFrame;
	mFrame.get_pointcloud(timestamp, pointcloud);
}

extern "C" void __declspec(dllexport) getPointCloud(uint64_t *timestamp, void **pointcloud) {
	captureIt captureit;
	captureit.getPointCloud(timestamp, pointcloud);
}
