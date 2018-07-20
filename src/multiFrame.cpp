//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18.
//

#include "multiFrame.hpp"

const int color_width = 1280;
const int color_height = 720;
const int color_fps = 30;
const int depth_width = 1280;
const int depth_height = 720;
const int depth_fps = 30;

void multiFrame::get_pointcloud(long *timestamp, void **pointcloud)
{
	std::lock_guard<std::mutex> guard(frames_mutex);
	*timestamp = time(0);

	if (numberOfCameras > 0) {
		*pointcloud = (reinterpret_cast<void *>(&merged_cloud));
		//boost::shared_ptr<PointCloud<PointXYZRGB>> merged = merge_views();
		//if (merged != NULL)
		//	*pointcloud = reinterpret_cast<void *> (&merge_views());
	}
	else {
		angle += 0.03;
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()));
		transformPointCloud(*merged_cloud, *rot_pc, transform);
		*pointcloud = (reinterpret_cast<void *>(&rot_pc));
	};
}

// Configure and initialize caputuring of one camera
void multiFrame::capture_start(const std::string& serial_number, multiFrame* m_frame, int camNum)
{
	std::thread([serial_number, m_frame, camNum]() mutable {

		std::cout << "starting camera ser no: " << serial_number << '\n';
		rs2::config cfg;
		cfg.enable_device(serial_number);
		cfg.enable_stream(RS2_STREAM_COLOR, color_width, color_height, RS2_FORMAT_RGB8, color_fps);
		cfg.enable_stream(RS2_STREAM_DEPTH, depth_width, depth_height, RS2_FORMAT_Z16, depth_fps);

		rs2::pipeline pipe;
		pipe.start(cfg);		// Start streaming with the configuration just set

								// Declare PointCloud object, for calculating PointClouds and texture mappings
		rs2::pointcloud pc;
		rs2::points points;

		while (true) // Application still alive?
		{
			// Wait for the next set of frames from the camera
			auto frames = pipe.wait_for_frames();
			auto depth = frames.get_depth_frame();
			auto color = frames.get_color_frame();
			float min = 100.f, max;

			// exclude pushing frames at this time
			std::lock_guard<std::mutex> guard(m_frame->frames_mutex);

			// Tell points frame to map to this color frame
			pc.map_to(color); // NB: This does not align the frames. That should be handled by setting resolution of cameras
			points = pc.calculate(depth);

			// Generate new vertices and color vector
			auto vertices = points.get_vertices();

			unsigned char *p = (unsigned char*)color.get_data();

			// Calculate the closest point distance
			for (int i = 0; i < points.size(); i++)
				if (vertices[i].z != 0 && min > vertices[i].z)
					min = vertices[i].z;

			// Set the maximum distance
			max = 0.8f + min;

			// Make PointCloud
			boost::shared_ptr<PointCloud<PointXYZRGB>> camera_pntcld(new PointCloud<PointXYZRGB>());

			for (int i = 0; i < points.size(); i++)
				if (min < vertices[i].z && vertices[i].z < max) { // "Background removal"
					PointXYZRGB pt;
					pt.x = vertices[i].x;
					pt.y = -vertices[i].y;
					pt.z = -vertices[i].z;
					int pi = i * 3;
					pt.r = p[pi];
					pt.g = p[pi + 1];
					pt.b = p[pi + 2];
					camera_pntcld->push_back(pt);
				}
			if (camera_pntcld->size() > 0)
				m_frame->push_pointcloud(camera_pntcld, camNum);
		}
		std::cout << "stopping camera ser no: " << serial_number << "\n";
		pipe.stop();
	}
	).detach();
}

void multiFrame::push_pointcloud(boost::shared_ptr<PointCloud<PointXYZRGB>> pntcld, const int camNum)
{
	if (camNum < 0 || camNum >= numberOfCameras) return;
	if (pntcld == NULL)  return;
	PointCloud_views[camNum] = pntcld;
	merged_cloud = pntcld; // TEMP
}

PointCloud<PointXYZRGB>::Ptr multiFrame::generate_pcl()
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

boost::shared_ptr<PointCloud<PointXYZRGB>> multiFrame::merge_views()
{
	/*
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << 0.5, 0.0, 0.0;
	transform.rotate(Eigen::AngleAxisf(1.4, Eigen::Vector3f::UnitY()));
	const Eigen::Affine3f trafo = transform;
	PointCloud<PointXYZRGB> *aligned_pntcld;
	for (const PointCloud<PointXYZRGB> *pc : PointCloud_views) {
	transformPointCloud(pc, aligned_pntcld, &trafo, true);
	}
	/**/
	if (PointCloud_views.size() > 0)
		return PointCloud_views[0];
	else
		return NULL;
}

void captureIt::getPointCloud(long *timestamp, void **pointcloud) {
	static multiFrame mFrame;
	mFrame.get_pointcloud(timestamp, pointcloud);
}

extern "C" void __declspec(dllexport) getPointCloud(long *timestamp, void **pointcloud) {
	captureIt captureit;
	captureit.getPointCloud(timestamp, pointcloud);
}
