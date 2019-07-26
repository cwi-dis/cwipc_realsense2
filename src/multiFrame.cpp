//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//

// Define to try and use hardware sync to synchronize multiple cameras
#define WITH_INTER_CAM_SYNC

// Define to enable optional dumping of RGB video frames (to test hardware sync)
#define WITH_DUMP_VIDEO_FRAMES

// Define to use polling for realsense camera data, otherwise use waiting
#undef WITH_POLLING

// Define to get (a little) debug prints
#undef CWIPC_DEBUG

// This is the dll source, so define external symbols as dllexport on windows.

#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif

#include <chrono>
#include <cstdint>
#include "cwipc_realsense2/multiFrame.hpp"
#include "cwipc_realsense2/api.h"
#include "cwipc_realsense2/utils.h"

#ifdef WITH_DUMP_VIDEO_FRAMES
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "cwipc_realsense2/stb_image_write.h"
#endif

MFCamera::MFCamera(rs2::context& ctx, MFConfigCapture& configuration, std::string _serial, std::string _usb)
:	serial(_serial),
	usb(_usb),
	pipe(ctx),
	do_depth_filtering(configuration.depth_filtering)
{
	// for an explanation of filtering see librealsense/doc/post-processing-filters.md and code in librealsense/src/proc
	if (do_depth_filtering) {
		dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, configuration.decimation_value);

		spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, configuration.spatial_iterations);
		spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, configuration.spatial_alpha);
		spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, configuration.spatial_delta);
		spat_filter.set_option(RS2_OPTION_HOLES_FILL, configuration.spatial_filling);

		temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, configuration.temporal_alpha);
		temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, configuration.temporal_delta);
		temp_filter.set_option(RS2_OPTION_HOLES_FILL, configuration.temporal_percistency);
	}
}

MFCamera::~MFCamera()
{
}

rs2::frameset MFCamera::get_frameset()
{
#ifdef WITH_POLLING
	// Poll to find if there is a next set of frames from the camera
	rs2::frameset frames;
	if (!pipe.poll_for_frames(&frames))
		return;
#else
	// Wait to find if there is a next set of frames from the camera
	rs2::frameset frames = pipe.wait_for_frames();
#endif
	return frames;
}

void MFCamera::process_depth_frame(rs2::depth_frame &depth) {
	if (do_depth_filtering) { // Apply filters
		//depth = dec_filter.process(depth);          // decimation filter
		depth = depth_to_disparity.process(depth);  // transform into disparity domain
		depth = spat_filter.process(depth);         // spatial filter
		depth = temp_filter.process(depth);         // temporal filter
		depth = disparity_to_depth.process(depth);  // revert back to depth domain
	}
}

// Configure and initialize caputuring of one camera
void MFCamera::start(MFConfigCapture& configuration)
{
	rs2::config cfg;
	if (is_usb3()) {
		std::cerr << "cwipc_realsense2: multiFrame: starting camera ser no: " << serial << " in usb3 mode\n";
		cfg.enable_device(serial);
		cfg.enable_stream(RS2_STREAM_COLOR, configuration.usb3_width, configuration.usb3_height, RS2_FORMAT_RGB8, configuration.usb3_fps);
		cfg.enable_stream(RS2_STREAM_DEPTH, configuration.usb3_width, configuration.usb3_height, RS2_FORMAT_Z16, configuration.usb3_fps);
	}
	else {
		std::cerr << "cwipc_realsense2: multiFrame: starting camera ser no: " << serial << " in usb2 mode\n";
		cfg.enable_device(serial);
		cfg.enable_stream(RS2_STREAM_COLOR, configuration.usb2_width, configuration.usb2_height, RS2_FORMAT_RGB8, configuration.usb2_fps);
		cfg.enable_stream(RS2_STREAM_DEPTH, configuration.usb2_width, configuration.usb2_height, RS2_FORMAT_Z16, configuration.usb2_fps);
	}
	pipe.start(cfg);		// Start streaming with the configuration just set
}

MFCapture::MFCapture(const char *_configFilename)
{
	if (_configFilename) {
		configFilename = _configFilename;
	}
	else {
		configFilename = "cameraconfig.xml";
	}
	// Create librealsense context for managing all connected RealSense devices
	auto devs = ctx.query_devices();
	const std::string platform_camera_name = "Platform Camera";
	//Sync messages to assign master and slave
#ifdef WITH_INTER_CAM_SYNC
	bool master_set = false;
	bool multiple_cameras = true; // xxxjack devs.size() > 1;
	rs2_error* e = nullptr;
#endif // WITH_INTER_CAM_SYNC
	// prepare storage for camera data for each connected camera
	for (auto dev : devs) {
		if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name) {
			boost::shared_ptr<Eigen::Affine3d> default_trafo(new Eigen::Affine3d());
			default_trafo->setIdentity();
			MFConfigCamera cd;
			cd.serial = std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
			cd.trafo = default_trafo;
			cd.cloud = new_cwipc_pcl_pointcloud();
			cd.background = { 0, 0, 0 };
			cd.cameraposition = { 0, 0, 0 };
			configuration.cameraConfig.push_back(cd);

			std::string camUsb(dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR));
			MFCamera rsd(ctx, configuration, cd.serial, camUsb);
			cameras.push_back(rsd);
#ifdef WITH_INTER_CAM_SYNC
			if (multiple_cameras) {
				auto allSensors = dev.query_sensors();
				bool foundSensorSupportingSync = false;
				for (auto sensor : allSensors) {
					if (sensor.supports(RS2_OPTION_INTER_CAM_SYNC_MODE)) {
						foundSensorSupportingSync = true;
						if (!master_set) {
							sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 1);
							master_set = true;
						} else {
							sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 2);
						}
					}
				}
				if (!foundSensorSupportingSync) {
					std::cerr << "cwipc_realsense2: multiFrame: Warning: camera " << cd.serial << " does not support inter-camera-sync";
				}
			}
#endif // WITH_INTER_CAM_SYNC
		}
	}

	if (configuration.cameraConfig.size() == 0) {
		// no camera connected, so we'll use a generated pointcloud instead
		generatedPC = generate_pcl();
		std::cerr << "cwipc_realsense2: multiFrame: No cameras found, default production is a spinning generated pointcloud of " << generatedPC->size() << " data points\n";
	}
	else {
		// Read the configuration
		if (!mf_file2config(configFilename.c_str(), &configuration)) {

			// the configuration file did not fully match the current situation so we have to update the admin
			std::vector<std::string> serials;
			std::vector<MFConfigCamera> realcams;

			// collect serial numbers of all connected cameras
			for (auto dev : devs) {
				if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name) {
					serials.push_back(std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)));
				}
			}
			
			// collect all camera's in the config that are connected
			for (MFConfigCamera cd : configuration.cameraConfig) {
				if ((find(serials.begin(), serials.end(), cd.serial) != serials.end()))
					realcams.push_back(cd);
				else
					std::cerr << "cwipc_realsense2: multiFrame: Warning: camera " << cd.serial << " is not connected\n";
			}
			// Reduce the active configuration to cameras that are connected
			configuration.cameraConfig = realcams;
		}
	}
	mergedPC = new_cwipc_pcl_pointcloud();

	// optionally set request for cwi_special_feature
	char* feature_request;
	feature_request = getenv("CWI_CAPTURE_FEATURE");
	if (feature_request != NULL)
		configuration.cwi_special_feature = feature_request;

	// find camerapositions
	for (int i = 0; i < configuration.cameraConfig.size(); i++) {
		cwipc_pcl_pointcloud pcptr(new_cwipc_pcl_pointcloud());
		cwipc_pcl_point pt;
		pt.x = 0;
		pt.y = 0;
		pt.z = 0;
		pcptr->push_back(pt);
		transformPointCloud(*pcptr, *pcptr, *configuration.cameraConfig[i].trafo);
		cwipc_pcl_point pnt = pcptr->points[0];
		configuration.cameraConfig[i].cameraposition.x = pnt.x;
		configuration.cameraConfig[i].cameraposition.y = pnt.y;
		configuration.cameraConfig[i].cameraposition.z = pnt.z;
	}


	// start the cameras
	for (auto cam: cameras)
		cam.start(configuration);
	starttime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

MFCapture::~MFCapture() {
	for (MFCamera rsd : cameras)
		rsd.pipe.stop();
	std::cerr << "cwipc_realsense2: multiFrame: stopped all camera's\n";
}



// API function that triggers the capture and returns the merged pointcloud and timestamp
cwipc_pcl_pointcloud MFCapture::get_pointcloud(uint64_t *timestamp)
{
	*timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	if (cameras.size() > 0) {
		for (int i = 0; i < cameras.size(); i++)
			camera_action(i, timestamp);

		if (merge_views()->size() > 0) {
#ifdef CWIPC_DEBUG
			std::cerr << "cwipc_realsense2: multiFrame: capturer produced a merged cloud of " << MergedPC->size() << " points in ringbuffer " << ring_index << "\n";
#endif
		}
		else {
#ifdef CWIPC_DEBUG
			std::cerr << "cwipc_realsense2: multiFrame: Warning: capturer did get an empty pointcloud\n\n";
#endif
			// HACK to make sure the encoder does not get an empty pointcloud 
			cwipc_pcl_point point;
			point.x = 1.0;
			point.y = 1.0;
			point.z = 1.0;
			point.rgb = 0.0;
			mergedPC->points.push_back(point);
		}
	}
	else {	// return a spinning generated mathematical pointcloud
		static float angle;
		angle += 0.031415;
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()));
		transformPointCloud(*generatedPC, *mergedPC, transform);
	}
	return mergedPC;
}

// return the merged cloud 
cwipc_pcl_pointcloud MFCapture::get_mostRecentPointCloud()
{
	return mergedPC;
}

// get new frames from the camera and update the pointcloud of the camera's data 
void MFCapture::camera_action(int camera_index, uint64_t *timestamp)
{
	MFCamera* rsd = &cameras[camera_index];
	MFConfigCamera* cd = &configuration.cameraConfig[camera_index];
	rs2::pointcloud pc;
	rs2::points points;

	rs2::frameset frames = rsd->get_frameset();

	rs2::depth_frame depth = frames.get_depth_frame();
	rs2::video_frame color = frames.get_color_frame();

	rsd->process_depth_frame(depth);
	
#ifdef WITH_DUMP_VIDEO_FRAMES
	// On special request write video to png
	if (configuration.cwi_special_feature == "dumpvideoframes") {
		std::stringstream png_file;
		png_file <<  "videoframe_" << *timestamp - starttime << "_" << camera_index << ".png";
		stbi_write_png(png_file.str().c_str(), color.get_width(), color.get_height(),
			color.get_bytes_per_pixel(), color.get_data(), color.get_stride_in_bytes());
	}
#endif // WITH_DUMP_VIDEO_FRAMES

	cd->cloud->clear();
	// Tell points frame to map to this color frame
	pc.map_to(color); // NB: This does not align the frames. That should be handled by setting resolution of cameras

	uint8_t camera_label = (uint8_t)1 << camera_index;
	points = pc.calculate(depth);

	// Generate new vertices and color vector
	auto vertices = points.get_vertices();

	unsigned char *colors = (unsigned char*)color.get_data();

	if (configuration.background_removal) {
		MFConfigCamera* cd = get_camera_config(rsd->serial);

		// Set the background removal window
        if (cd->background.z > 0.0) {
            rsd->maxz = cd->background.z;
			rsd->minz = 0.0;
			if (cd->background.x != 0.0) {
				rsd->minx = cd->background.x;
			}
			else {
				for (int i = 0; i < points.size(); i++) {
					double minz = 100;
					if (vertices[i].z != 0 && minz > vertices[i].z) {
						rsd->minz = vertices[i].z;
						rsd->minx = vertices[i].x;
					}
				}
			}
        }
		else {
			rsd->minz = 100.0;
			for (int i = 0; i < points.size(); i++) {
				if (vertices[i].z != 0 && rsd->minz > vertices[i].z) {
					rsd->minz = vertices[i].z;
					rsd->minx = vertices[i].x;
				}
			}
			rsd->maxz = 0.8f + rsd->minz;
		}

		// Make PointCloud
		for (int i = 0; i < points.size(); i++) {
			double x = rsd->minx - vertices[i].x; x *= x;
			double z = vertices[i].z;
			if (rsd->minz < z && z < rsd->maxz - x) { // Simple background removal, horizontally parabolic, vertically straight.
				cwipc_pcl_point pt;
				pt.x = vertices[i].x;
				pt.y = -vertices[i].y;
				pt.z = -z;
				int pi = i * 3;
				pt.r = colors[pi];
				pt.g = colors[pi + 1];
				pt.b = colors[pi + 2];
				pt.a = camera_label;
				if (!configuration.greenscreen_removal || mf_noChromaRemoval(&pt)) // chromakey removal
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
			pt.a = camera_label;
			if (!configuration.greenscreen_removal || mf_noChromaRemoval(&pt)) // chromakey removal
				cd->cloud->push_back(pt);
		}
	}
}

cwipc_pcl_pointcloud MFCapture::merge_views()
{
	cwipc_pcl_pointcloud aligned_cld(new_cwipc_pcl_pointcloud());
	mergedPC->clear();
	for (MFConfigCamera cd : configuration.cameraConfig) {
		cwipc_pcl_pointcloud cam_cld = cd.cloud;

		if (cam_cld->size() > 0) {
			transformPointCloud(*cam_cld, *aligned_cld, *cd.trafo);
			*mergedPC += *aligned_cld;
		}
	}

	if (configuration.cloud_resolution > 0) {
#ifdef CWIPC_DEBUG
		std::cerr << "cwipc_realsense2: multiFrame: Points before reduction: " << cloud_ptr.get()->size() << endl;
#endif
		pcl::VoxelGrid<cwipc_pcl_point> grd;
		grd.setInputCloud(mergedPC);
		grd.setLeafSize(configuration.cloud_resolution, configuration.cloud_resolution, configuration.cloud_resolution);
		grd.setSaveLeafLayout(true);
		grd.filter(*mergedPC);

#ifdef CWIPC_DEBUG
		std::cerr << "cwipc_realsense2: multiFrame: Points after reduction: " << cloud_ptr.get()->size() << endl;
#endif
	}
	return mergedPC;
}

MFConfigCamera* MFCapture::get_camera_config(std::string serial) {
	for (int i = 0; i < configuration.cameraConfig.size(); i++)
		if (configuration.cameraConfig[i].serial == serial)
			return &configuration.cameraConfig[i];
	return NULL;
}

MFCamera* MFCapture::get_camera(std::string serial) {
	for (int i = 0; i < cameras.size(); i++)
		if (cameras[i].serial == serial)
			return &cameras[i];
	return NULL;
}

// generate a mathematical pointcloud
cwipc_pcl_pointcloud MFCapture::generate_pcl()
{
	cwipc_pcl_pointcloud point_cloud_ptr(new_cwipc_pcl_pointcloud());
	uint8_t r(255), g(15), b(15);

	for (float z(-1.0f); z <= 1.0f; z += 0.005f) {
        float angle(0.0);
		while (angle <= 360.0) {
			cwipc_pcl_point point;
			point.x = 0.5f*cosf(pcl::deg2rad(angle))*(1.0f - z * z);
			point.y = sinf(pcl::deg2rad(angle))*(1.0f - z * z);
			point.z = z;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
			point.rgb = *reinterpret_cast<float*>(&rgb);
			point_cloud_ptr->points.push_back(point);
            float r = sqrt(point.x*point.x + point.y*point.y);
            if (r > 0.0)
                angle += 0.27/r;
            else break;
		}
		if (z < 0.0) { r -= 1; g += 1; }
		else { g -= 1; b += 1; }
	}
	point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
	point_cloud_ptr->height = 1;
	return point_cloud_ptr;
}

