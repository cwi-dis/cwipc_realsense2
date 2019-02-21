//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//

#include <chrono>
#include <cstdint>

// This is the dll source, so define external symbols as dllexport on windows.

#if defined(WIN32) || defined(_WIN32)
#define CWIPC_DLL_ENTRY __declspec(dllexport)
#endif

#include "cwipc_realsense/multiFrame.hpp"
#include "cwipc_realsense/utils.h"
#include "cwipc_realsense/api.h"

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

multiFrame::multiFrame() {
	// Create librealsense context for managing all connected RealSense devices
	rs2::context ctx;
	auto devs = ctx.query_devices();

	const std::string platform_camera_name = "Platform Camera";

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
			Configuration.camera_data.push_back(*cc);
			camera_start(*cc);
		}
	}
	if (Configuration.camera_data.size() == 0) {
		// no camera connected, so we'll use a generated pointcloud instead
		GeneratedPC = generate_pcl();
		cout << "No cameras found, default production is a spinning generated pointcloud of " << GeneratedPC->size() << " data points\n";
	}
	else {
		// Read the configuration
		if (!file2config("cameraconfig.xml", &Configuration)) {
			// check and remove cameras that are not connected
			vector<string> serials;
			for (auto dev : devs) {
				if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name) {
					serials.push_back(string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)));
				}
			}
			vector<cameradata> realcams;
			for (cameradata cd : Configuration.camera_data) {
				if ((find(serials.begin(), serials.end(), cd.serial) != serials.end()))
					realcams.push_back(cd);
				else
					cout << "\tcamera " << cd.serial << " is not connected\n";
			}
			Configuration.camera_data = realcams;
		}
	}

	// prepare ringbuffer
	for (int i = 0; i < Configuration.ringbuffer_size; i++) {
		boost::shared_ptr<PointCloudT> buf(new PointCloudT());
		RingBuffer.push_back(buf);
	}
}

multiFrame::~multiFrame() {
	for (cameradata cd : Configuration.camera_data) {
		cd.pipe.stop();
	}
	cout << "stopped all camera's\n";
}

// API function that triggers the capture and returns the merged pointcloud and timestamp
void multiFrame::get_pointcloud(uint64_t *timestamp, void **pointcloud)
{
	*timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

	if (Configuration.camera_data.size() > 0) {
		for (cameradata cd : Configuration.camera_data)
			camera_action(cd);
		merge_views(RingBuffer[ring_index]);

		if (RingBuffer[ring_index].get()->size() > 0) {
#ifdef CWIPC_DEBUG
			cout << "capturer produced a merged cloud of " << RingBuffer[ring_index].get()->size() << " points in ringbuffer " << ring_index << "\n";
#endif
			*pointcloud = reinterpret_cast<void *> (&(RingBuffer[ring_index]));
		}
		else {
#ifdef CWIPC_DEBUG
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
		static float angle;
		angle += 0.031415;
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()));
		transformPointCloud(*GeneratedPC, *RingBuffer[ring_index], transform);
		*pointcloud = reinterpret_cast<void *> (&(RingBuffer[ring_index]));
	}
	ring_index = ring_index < Configuration.ringbuffer_size - 1 ? ++ring_index : 0;
}

// return the merged cloud 
boost::shared_ptr<PointCloudT> multiFrame::getPointCloud()
{
	return RingBuffer[ring_index];
}

// Configure and initialize caputuring of one camera
void multiFrame::camera_start(cameradata cd)
{
	cout << "starting camera ser no: " << cd.serial << '\n';

	rs2::config cfg;
	cfg.enable_device(cd.serial);
	cfg.enable_stream(RS2_STREAM_COLOR, color_width, color_height, RS2_FORMAT_RGB8, color_fps);
	cfg.enable_stream(RS2_STREAM_DEPTH, depth_width, depth_height, RS2_FORMAT_Z16, depth_fps);

	cd.pipe.start(cfg);		// Start streaming with the configuration just set
}

// get new frames from the camera and update the pointcloud of the camera's data 
void multiFrame::camera_action(cameradata cd)
{
	rs2::pointcloud pc;
	rs2::points points;

#ifdef POLLING
	// Poll to find if there is a next set of frames from the camera
	rs2::frameset frames;
	if (!cd.pipe.poll_for_frames(&frames))
		return;
#else
	// Wait to find if there is a next set of frames from the camera
	auto frames = cd.pipe.wait_for_frames();
#endif

	auto depth = frames.get_depth_frame();
	auto color = frames.get_color_frame();

	cd.cloud->clear();

	// Tell points frame to map to this color frame
	pc.map_to(color); // NB: This does not align the frames. That should be handled by setting resolution of cameras
	points = pc.calculate(depth);

	// Generate new vertices and color vector
	auto vertices = points.get_vertices();

	unsigned char *colors = (unsigned char*)color.get_data();

	if (Configuration.background_removal) {
		float minz = 100.0f, maxz, minx;

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
				if (!Configuration.greenscreen_removal || noChromaRemoval(&pt)) // chromakey removal
					cd.cloud->push_back(pt);
			}
		}
	}
	else {
		// Make PointCloud
		for (int i = 0; i < points.size(); i++) {
			PointT pt;
			pt.x = vertices[i].x;
			pt.y = -vertices[i].y;
			pt.z = -vertices[i].z;
			int pi = i * 3;
			pt.r = colors[pi];
			pt.g = colors[pi + 1];
			pt.b = colors[pi + 2];
			if (!Configuration.greenscreen_removal || noChromaRemoval(&pt)) // chromakey removal
				cd.cloud->push_back(pt);
		}
	}
}

void multiFrame::merge_views(boost::shared_ptr<PointCloudT> cloud_ptr)
{
	PointCloudT::Ptr aligned_cld(new PointCloudT);
	cloud_ptr->clear();
	for (cameradata cd : Configuration.camera_data) {
		PointCloudT *cam_cld = cd.cloud.get();
		if (cam_cld->size() > 0) {
			transformPointCloud(*cam_cld, *aligned_cld, *cd.trafo);
			*cloud_ptr.get() += *aligned_cld;
		}
	}

	if (Configuration.cloud_resolution > 0) {
#ifdef CWIPC_DEBUG
		cout << "Points before reduction: " << cloud_ptr.get()->size() << endl;
#endif
		VoxelGrid<PointT> grd;
		grd.setInputCloud(cloud_ptr);
		grd.setLeafSize(Configuration.cloud_resolution, Configuration.cloud_resolution, Configuration.cloud_resolution);
		grd.setSaveLeafLayout(true);
		grd.filter(*cloud_ptr);
#ifdef CWIPC_DEBUG
		cout << "Points after reduction: " << cloud_ptr.get()->size() << endl;
#endif
	}
}

// generate a mathematical pointcloud
PointCloudT::Ptr multiFrame::generate_pcl()
{
	PointCloudT::Ptr point_cloud_ptr(new PointCloudT);
	uint8_t r(255), g(15), b(15);
	for (float z(-1.0f); z <= 1.0f; z += 0.005f) {
		for (float angle(0.0); angle <= 360.0; angle += 1.0f) {
			PointT point;
			point.x = 0.5f*cosf(deg2rad(angle))*(1.0f - z * z);
			point.y = sinf(deg2rad(angle))*(1.0f - z * z);
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


///////////////////////////
// class captureIt stuff //
///////////////////////////

void captureIt::getPointCloud(uint64_t *timestamp, void **pointcloud) {
	static multiFrame mFrame;

#ifdef CWIPC_DEBUG
	cout << "captureIt is asked for a pointcloud\n";
#endif

	mFrame.get_pointcloud(timestamp, pointcloud);

#ifdef CWIPC_DEBUG
	boost::shared_ptr<PointCloudT> captured_pc;
	captured_pc = *reinterpret_cast<boost::shared_ptr<PointCloudT>*>(*pointcloud);
	cout << "captureIt handed over a pointcloud of " << captured_pc.get()->size() << " points\n";
	cout.flush();
#endif
}

// C-compatible entry point

void  getPointCloud(uint64_t *timestamp, void **pointcloud) {
	captureIt captureit;
	captureit.getPointCloud(timestamp, pointcloud);
}
