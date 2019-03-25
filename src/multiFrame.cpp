//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//

#include <chrono>
#include <cstdint>

// This is the dll source, so define external symbols as dllexport on windows.

#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif

#include "cwipc_realsense2/multiFrame.hpp"
#include "cwipc_realsense2/api.h"
#include "cwipc_realsense2/utils.h"

multiFrame::multiFrame() {
	// Create librealsense context for managing all connected RealSense devices
	rs2::context ctx;
	auto devs = ctx.query_devices();

	const std::string platform_camera_name = "Platform Camera";

	// prepare storage for camera data for each connected camera
	for (auto dev : devs) {
		if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name) {
			boost::shared_ptr<Eigen::Affine3d> default_trafo(new Eigen::Affine3d());
			default_trafo->setIdentity();
			cameradata cd;
			cd.serial = std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
			cd.usb = std::string(dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR));
			cd.cloud = new_cwipc_pcl_pointcloud();
			cd.trafo = default_trafo;
			configuration.camera_data.push_back(cd);
		}
	}

	if (configuration.camera_data.size() == 0) {
		// no camera connected, so we'll use a generated pointcloud instead
		GeneratedPC = generate_pcl();
		std::cout << "No cameras found, default production is a spinning generated pointcloud of " << GeneratedPC->size() << " data points\n";
	}
	else {
		// Read the configuration
		if (!file2config("cameraconfig.xml", &configuration)) {

			// the configuration file did not fully match the current situation so we have to update the admin
			std::vector<std::string> serials;
			std::vector<cameradata> realcams;

			// collect serial numbers of all connected cameras
			for (auto dev : devs) {
				if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name) {
					serials.push_back(std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)));
				}
			}
			
			// collect all camera's in the config that are connected
			for (cameradata cd : configuration.camera_data) {
				if ((find(serials.begin(), serials.end(), cd.serial) != serials.end()))
					realcams.push_back(cd);
				else
					std::cout << "WARNING: camera " << cd.serial << " is not connected\n";
			}
			// Reduce the active configuration to cameras that are connected
			configuration.camera_data = realcams;
		}
	}
	MergedPC = new_cwipc_pcl_pointcloud();

	// for an explanation of filtering see librealsense/doc/post-processing-filters.md and code in librealsense/src/proc 
	dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, configuration.decimation_value);

	spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, configuration.spatial_iterations);
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, configuration.spatial_alpha);
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, configuration.spatial_delta);
	spat_filter.set_option(RS2_OPTION_HOLES_FILL, configuration.spatial_filling);

	temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, configuration.temporal_alpha);
	temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, configuration.temporal_delta);
	temp_filter.set_option(RS2_OPTION_HOLES_FILL, configuration.temporal_percistency);

	// start the cameras
	for (int i = 0; i < configuration.camera_data.size(); i++)
		camera_start(&(configuration.camera_data[i]));
}

multiFrame::~multiFrame() {
	for (cameradata cd : configuration.camera_data) {
		cd.pipe.stop();
	}
	std::cout << "stopped all camera's\n";
}

#if 0
// API function that triggers the capture and returns the merged pointcloud and timestamp
void multiFrame::get_pointcloud(uint64_t *timestamp, void **pointcloud)
{
	*timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

	if (Configuration.camera_data.size() > 0) {
		for (cameradata cd : Configuration.camera_data)
			camera_action(cd);
		merge_views();

		if (MergedPC.get()->size() > 0) {
#ifdef CWIPC_DEBUG
			std::cout << "capturer produced a merged cloud of " << MergedPC->size() << " points in ringbuffer " << ring_index << "\n";
#endif
			*pointcloud = reinterpret_cast<void *> (&(MergedPC));
		}
		else {
#ifdef CWIPC_DEBUG
			std::cout << "\nWARNING: capturer did get an empty pointcloud\n\n";
#endif
			// HACK to make sure the encoder does not get an empty pointcloud 
			cwipc_pcl_point point;
			point.x = 1.0;
			point.y = 1.0;
			point.z = 1.0;
			point.rgb = 0.0;
			MergedPC->points.push_back(point);
			*pointcloud = reinterpret_cast<void *> (&(MergedPC));
		}
	}
	else {	// return a spinning generated mathematical pointcloud
		static float angle;
		angle += 0.031415;
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()));
		transformPointCloud(*GeneratedPC, *MergedPC, transform);
		*pointcloud = reinterpret_cast<void *> (&(MergedPC));
	}
	ring_index = ring_index < Configuration.ringbuffer_size - 1 ? ++ring_index : 0;
}
#else
// API function that triggers the capture and returns the merged pointcloud and timestamp
cwipc_pcl_pointcloud multiFrame::get_pointcloud(uint64_t *timestamp)
{
	*timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	if (configuration.camera_data.size() > 0) {
		for (int i = 0; i < configuration.camera_data.size(); i++) {
			camera_action(&configuration.camera_data[i]);
		}

		if (merge_views()->size() > 0) {
#ifdef DEBUG
			std::cout << "capturer produced a merged cloud of " << MergedPC->size() << " points in ringbuffer " << ring_index << "\n";
#endif
		}
		else {
#ifdef DEBUG
			std::cout << "\nWARNING: capturer did get an empty pointcloud\n\n";
#endif
			// HACK to make sure the encoder does not get an empty pointcloud 
			cwipc_pcl_point point;
			point.x = 1.0;
			point.y = 1.0;
			point.z = 1.0;
			point.rgb = 0.0;
			MergedPC->points.push_back(point);
		}
	}
	else {	// return a spinning generated mathematical pointcloud
		static float angle;
		angle += 0.031415;
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()));
		transformPointCloud(*GeneratedPC, *MergedPC, transform);
	}
	return MergedPC;
}

#endif

// return the merged cloud 
cwipc_pcl_pointcloud multiFrame::getPointCloud()
{
	return MergedPC;
}

// Configure and initialize caputuring of one camera
void multiFrame::camera_start(cameradata* cd)
{
	rs2::config cfg;
	if (cd->usb[0] == '3') {
		std::cout << "starting camera ser no: " << cd->serial << " in usb3 mode\n";
		cfg.enable_device(cd->serial);
		cfg.enable_stream(RS2_STREAM_COLOR, configuration.usb3_width, configuration.usb3_height, RS2_FORMAT_RGB8, configuration.usb3_fps);
		cfg.enable_stream(RS2_STREAM_DEPTH, configuration.usb3_width, configuration.usb3_height, RS2_FORMAT_Z16, configuration.usb3_fps);
	}
	else {
		std::cout << "starting camera ser no: " << cd->serial << " in usb2 mode\n";
		cfg.enable_device(cd->serial);
		cfg.enable_stream(RS2_STREAM_COLOR, configuration.usb2_width, configuration.usb2_height, RS2_FORMAT_RGB8, configuration.usb2_fps);
		cfg.enable_stream(RS2_STREAM_DEPTH, configuration.usb2_width, configuration.usb2_height, RS2_FORMAT_Z16, configuration.usb2_fps);
	}
	cd->pipe.start(cfg);		// Start streaming with the configuration just set
}

// get new frames from the camera and update the pointcloud of the camera's data 
void multiFrame::camera_action(cameradata* cd)
{
	rs2::pointcloud pc;
	rs2::points points;

#ifdef POLLING
	// Poll to find if there is a next set of frames from the camera
	rs2::frameset frames;
	if (!cd->pipe.poll_for_frames(&frames))
		return;
#else
	// Wait to find if there is a next set of frames from the camera
	rs2::frameset frames = cd->pipe.wait_for_frames();
#endif

	rs2::depth_frame depth = frames.get_depth_frame();
	rs2::video_frame color = frames.get_color_frame();

	//std::cout << "size " << depth.get_height() << ", " << depth.get_width();
	
	//std::cout << " disp " << depth.get_height() << ", " << depth.get_width() << "\n";
	cd->cloud->clear();

	// Tell points frame to map to this color frame
	pc.map_to(color); // NB: This does not align the frames. That should be handled by setting resolution of cameras
	
	if (configuration.depth_filtering) {    // Apply filters
		//depth = dec_filter.process(depth);			// decimation filter
		depth = depth_to_disparity.process(depth);	// transform into disparity domain
		depth = spat_filter.process(depth);			// spatial filter
		depth = temp_filter.process(depth);			// temporal filter
		depth = disparity_to_depth.process(depth);	// revert back to depth domain
	}
	points = pc.calculate(depth);

	// Generate new vertices and color vector
	auto vertices = points.get_vertices();

	unsigned char *colors = (unsigned char*)color.get_data();

	if (configuration.background_removal) {

		// Set the background removal window
        if (cd->background_z > 0.0) {
            cd->maxz = cd->background_z;
            cd->minz = 0.0;
			if (cd->background_x != 0.0) {
				cd->minx = cd->background_x;
			}
			else {
				for (int i = 0; i < points.size(); i++) {
					double minz = 100;
					if (vertices[i].z != 0 && minz > vertices[i].z) {
						minz = vertices[i].z;
						cd->minx = vertices[i].x;
					}
				}
			}
        }
		else {
			cd->minz = 100.0;
			for (int i = 0; i < points.size(); i++) {
				if (vertices[i].z != 0 && cd->minz > vertices[i].z) {
					cd->minz = vertices[i].z;
					cd->minx = vertices[i].x;
				}
			}
			cd->maxz = 0.8f + cd->minz;
		}

		// Make PointCloud
		for (int i = 0; i < points.size(); i++) {
			double x = cd->minx - vertices[i].x; x *= x;
			double z = vertices[i].z;
			if (cd->minz < z && z < cd->maxz - x) { // Simple background removal, horizontally parabolic, vertically straight.
				cwipc_pcl_point pt;
				pt.x = vertices[i].x;
				pt.y = -vertices[i].y;
				pt.z = -z;
				int pi = i * 3;
				pt.r = colors[pi];
				pt.g = colors[pi + 1];
				pt.b = colors[pi + 2];
				if (!configuration.greenscreen_removal || noChromaRemoval(&pt)) // chromakey removal
					cd->cloud->push_back(pt);
			}
		}
	}
	else {
		// Make PointCloud
		for (int i = 0; i < points.size(); i++) {
			cwipc_pcl_point pt;

			pt.x = vertices[i].x;
			pt.y = -vertices[i].y;
			pt.z = -vertices[i].z;
			int pi = i * 3;
			pt.r = colors[pi];
			pt.g = colors[pi + 1];
			pt.b = colors[pi + 2];
			if (!configuration.greenscreen_removal || noChromaRemoval(&pt)) // chromakey removal
				cd->cloud->push_back(pt);
		}
	}
}

cwipc_pcl_pointcloud multiFrame::merge_views()
{
	cwipc_pcl_pointcloud aligned_cld(new_cwipc_pcl_pointcloud());
	MergedPC->clear();
	for (cameradata cd : configuration.camera_data) {
		cwipc_pcl_pointcloud cam_cld = cd.cloud;

		if (cam_cld->size() > 0) {
			transformPointCloud(*cam_cld, *aligned_cld, *cd.trafo);
			*MergedPC += *aligned_cld;
		}
	}

	if (configuration.cloud_resolution > 0) {
#ifdef CWIPC_DEBUG
		std::cout << "Points before reduction: " << cloud_ptr.get()->size() << endl;
#endif
		pcl::VoxelGrid<cwipc_pcl_point> grd;
		grd.setInputCloud(MergedPC);
		grd.setLeafSize(configuration.cloud_resolution, configuration.cloud_resolution, configuration.cloud_resolution);
		grd.setSaveLeafLayout(true);
		grd.filter(*MergedPC);

#ifdef DEBUG
		std::cout << "Points after reduction: " << cloud_ptr.get()->size() << endl;
#endif
	}
	return MergedPC;
}

// generate a mathematical pointcloud
cwipc_pcl_pointcloud
multiFrame::generate_pcl()
{
	cwipc_pcl_pointcloud point_cloud_ptr(new_cwipc_pcl_pointcloud());
	uint8_t r(255), g(15), b(15);
    
    cwipc_pcl_point pmin;
    pmin.x = 0.0f;
    pmin.y = 0.0f;
    pmin.z = -1.0f;
    uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
    pmin.rgb = *reinterpret_cast<float*>(&rgb);
    point_cloud_ptr->points.push_back(pmin);

	for (float z(-0.995f); z <= 0.995f; z += 0.005f) {
		for (float angle(0.0); angle <= 360.0; angle += 1.0f) {
			cwipc_pcl_point point;
			point.x = 0.5f*cosf(pcl::deg2rad(angle))*(1.0f - z * z);
			point.y = sinf(pcl::deg2rad(angle))*(1.0f - z * z);
			point.z = z;
            rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
			point.rgb = *reinterpret_cast<float*>(&rgb);
			point_cloud_ptr->points.push_back(point);
		}
		if (z < 0.0) { r -= 1; g += 1; }
		else { g -= 1; b += 1; }
	}
    cwipc_pcl_point pmax;
    pmax.x = 0.0f;
    pmax.y = 0.0f;
    pmax.z = 1.0f;
    rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
    pmax.rgb = *reinterpret_cast<float*>(&rgb);
    point_cloud_ptr->points.push_back(pmax);

	point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
	point_cloud_ptr->height = 1;
	return point_cloud_ptr;
}

