//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18.
//
#include <chrono>
#include <cstdint>
#include "multiFrame.hpp"

const int color_width = 1280;
const int color_height = 720;
const int color_fps = 30;
const int depth_width = 1280;
const int depth_height = 720;
const int depth_fps = 30;

// Configure and initialize caputuring of one camera
void multiFrame::capture_start(const std::string& serial_number, multiFrame* m_frame, int camNum)
{
	std::thread([serial_number, m_frame, camNum]() mutable {
		rs2::config cfg;
		rs2::pipeline pipe;
		rs2::pointcloud pc;
		rs2::points points;

		cfg.enable_device(serial_number);
		cfg.enable_stream(RS2_STREAM_COLOR, color_width, color_height, RS2_FORMAT_RGB8, color_fps);
		cfg.enable_stream(RS2_STREAM_DEPTH, depth_width, depth_height, RS2_FORMAT_Z16, depth_fps);

		pipe.start(cfg);		// Start streaming with the configuration just set
		std::cout << "started camera ser no: " << serial_number << '\n';

		Eigen::Affine3f transform = Eigen::Affine3f::Identity();

		// ================ START TEMP SETTING OF MATRICES ================
		if (camNum > 0) {
			transform.translation() << 0.0, 0.0, 0.0;
			transform.rotate(Eigen::AngleAxisf(camNum*0.7, Eigen::Vector3f::UnitY()));
		}
		// ================= END TEMP SETTING OF MATRICES =================

		// store transformation in vector
		m_frame->MergeTrafos.push_back(transform);
		// simultaneously prepare storage of camera views
		boost::shared_ptr<PointCloud<PointXYZRGB>> dummy_pntcld(new PointCloud<PointXYZRGB>());
		m_frame->PointCloudViews.push_back(dummy_pntcld);

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

// store the pointcloud of a camera in a vector of all individual camera views
void multiFrame::push_pointcloud(boost::shared_ptr<PointCloud<PointXYZRGB>> pntcld, const int camNum)
{
	if (camNum < 0 || camNum >= numberOfCameras) return;
	if (pntcld == NULL)  return;
	PointCloudViews[camNum] = pntcld;
}

boost::shared_ptr<PointCloud<PointXYZRGB>> multiFrame::merge_views()
{
	PointCloud<PointXYZRGB> *aligned_pntcld = new PointCloud<PointXYZRGB>();

	if (MergedCloud)
		MergedCloud.get()->clear();

	if (numberOfCameras == 1)
		return PointCloudViews[0];

	for (int i = 0; i < numberOfCameras; i++) {
		Eigen::Affine3f transform = MergeTrafos[i];
		transformPointCloud(*PointCloudViews[i].get(), *aligned_pntcld, transform);
		if (aligned_pntcld->size() <= 0) continue;
		for (PointXYZRGB pnt : *aligned_pntcld)
			MergedCloud->push_back(pnt);
	}
	aligned_pntcld->clear();
	delete aligned_pntcld;

	return MergedCloud;
}

void multiFrame::get_pointcloud(uint64_t *timestamp, void **pointcloud)
{
	std::lock_guard<std::mutex> guard(frames_mutex);
	*timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

	if (numberOfCameras > 0) {
		//*pointcloud = (reinterpret_cast<void *>(&MergedCloud));
		boost::shared_ptr<PointCloud<PointXYZRGB>> merged = merge_views();
		if (merged != NULL)
			*pointcloud = reinterpret_cast<void *> (&merge_views());
	}
	else {	// return a spinning generated mathematical pointcloud
		angle += 0.03;
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()));
		transformPointCloud(*MergedCloud, *RotatedPC, transform);
		*pointcloud = (reinterpret_cast<void *>(&RotatedPC));
	};
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
