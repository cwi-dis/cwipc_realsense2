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
#undef CWIPC_DEBUG_THREAD

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

// Static variable used to print a warning message when we re-create an MFCapture
// if there is another one open.
static int numberOfCapturersActive = 0;

MFCamera::MFCamera(rs2::context& ctx, MFCaptureConfig& configuration, int _camera_index, MFCameraData& _camData, std::string _usb)
:	minx(0), minz(0), maxz(0),
	camera_index(_camera_index),
	serial(_camData.serial),
	camData(_camData),
	usb(_usb),
	pipe(ctx),
	do_depth_filtering(configuration.depth_filtering),
	do_background_removal(configuration.background_removal),
	do_greenscreen_removal(configuration.greenscreen_removal),
	stopped(true),
	grabber_thread(nullptr),
	captured_frame_queue(1),
	processing_frame_queue(1)
{
#ifdef CWIPC_DEBUG
		std::cout << "MFCapture: creating camera " << serial << std::endl;
#endif
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
#ifdef CWIPC_DEBUG
	std::cout << "MFCamera: destroying " << serial << std::endl;
#endif
	assert(stopped);
}

void MFCamera::capture_frameset()
{
	current_frameset = captured_frame_queue.wait_for_frame();
}

// Configure and initialize caputuring of one camera
void MFCamera::start(MFCaptureConfig& configuration)
{
	assert(stopped);
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

void MFCamera::stop()
{
	assert(!stopped);
	assert(grabber_thread);
	stopped = true;
	grabber_thread->join();
	processing_thread->join();
	pipe.stop();
}

void MFCamera::start_capturer()
{
	assert(stopped);
	stopped = false;
	grabber_thread = new std::thread(&MFCamera::_capture_thread_main, this);
	processing_thread = new std::thread(&MFCamera::_processing_thread_main, this);
}

void MFCamera::_capture_thread_main()
{
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "frame capture: cam=" << serial << " thread started" << std::endl;
#endif
	while(!stopped) {
		// Wait to find if there is a next set of frames from the camera
		rs2::frameset frames = pipe.wait_for_frames();
#ifdef CWIPC_DEBUG_THREAD
		std::cerr << "frame capture: cam=" << serial << ", seq=" << frames.get_frame_number() << std::endl;
#endif
		captured_frame_queue.enqueue(frames);
		std::this_thread::yield();
	}
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "frame capture: cam=" << serial << " thread stopped" << std::endl;
#endif
}

void MFCamera::_processing_thread_main()
{
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "frame processing: cam=" << serial << " thread started" << std::endl;
#endif
	while(!stopped) {
		// Wait for next frame to process. Allow aborting in case of stopped becoming false...
		rs2::frameset processing_frameset;
		bool ok = processing_frame_queue.try_wait_for_frame(&processing_frameset, 1000);
		if (!ok) continue;

		std::lock_guard<std::mutex> lock(processing_mutex);

		rs2::depth_frame depth = processing_frameset.get_depth_frame();
		rs2::video_frame color = processing_frameset.get_color_frame();
#ifdef CWIPC_DEBUG
		std::cerr << "frame processing: cam=" << serial << ", depthseq=" << depth.get_frame_number() << ", colorseq=" << depth.get_frame_number() << std::endl;
#endif
		_process_depth_frame(depth);
		camData.cloud->clear();
		// Tell points frame to map to this color frame
		rs2::pointcloud pc;
		rs2::points points;
		pc.map_to(color); // NB: This does not align the frames. That should be handled by setting resolution of cameras

		uint8_t camera_label = (uint8_t)1 << camera_index;
		points = pc.calculate(depth);

		// Generate new vertices and color vector
		auto vertices = points.get_vertices();

		unsigned char *colors = (unsigned char*)color.get_data();

		if (do_background_removal) {

			// Set the background removal window
			if (camData.background.z > 0.0) {
				maxz = camData.background.z;
				minz = 0.0;
				if (camData.background.x != 0.0) {
					minx = camData.background.x;
				}
				else {
					for (int i = 0; i < points.size(); i++) {
						double minz = 100;
						if (vertices[i].z != 0 && minz > vertices[i].z) {
							minz = vertices[i].z;
							minx = vertices[i].x;
						}
					}
				}
			}
			else {
				minz = 100.0;
				for (int i = 0; i < points.size(); i++) {
					if (vertices[i].z != 0 && minz > vertices[i].z) {
						minz = vertices[i].z;
						minx = vertices[i].x;
					}
				}
				maxz = 0.8f + minz;
			}
			// Pre-allocate space in the pointcloud (so we don't realloc).
			// Note that we allocate too much space because the Z filtering will remove points.
			camData.cloud->reserve(points.size());
			// Make PointCloud
			for (int i = 0; i < points.size(); i++) {
				double x = minx - vertices[i].x; x *= x;
				double z = vertices[i].z;
				if (minz < z && z < maxz - x) { // Simple background removal, horizontally parabolic, vertically straight.
					cwipc_pcl_point pt;
					pt.x = vertices[i].x;
					pt.y = -vertices[i].y;
					pt.z = -z;
					int pi = i * 3;
					pt.r = colors[pi];
					pt.g = colors[pi + 1];
					pt.b = colors[pi + 2];
					pt.a = camera_label;
					if (!do_greenscreen_removal || mf_noChromaRemoval(&pt)) // chromakey removal
						camData.cloud->push_back(pt);
				}
			}
		}
		else {
			// Pre-allocate space in the pointcloud (so we don't realloc)
			camData.cloud->reserve(points.size());
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
				if (!do_greenscreen_removal || mf_noChromaRemoval(&pt)) // chromakey removal
					camData.cloud->push_back(pt);
			}
		}
		// Notify wait_for_pc that we're done.
		processing_done = true;
		processing_done_cv.notify_one();
	}
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "frame processing: cam=" << serial << " thread stopped" << std::endl;
#endif
}

void MFCamera::_process_depth_frame(rs2::depth_frame &depth) {
	if (do_depth_filtering) { // Apply filters
		//depth = dec_filter.process(depth);          // decimation filter
		depth = depth_to_disparity.process(depth);  // transform into disparity domain
		depth = spat_filter.process(depth);         // spatial filter
		depth = temp_filter.process(depth);         // temporal filter
		depth = disparity_to_depth.process(depth);  // revert back to depth domain
	}
}

void MFCamera::create_pc_from_frames()
{
	processing_frame_queue.enqueue(current_frameset);
}

void MFCamera::wait_for_pc()
{
	std::unique_lock<std::mutex> lock(processing_mutex);
	processing_done_cv.wait(lock, [this]{ return processing_done; });
	processing_done = false;
}

uint64_t MFCamera::get_capture_timestamp()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void
MFCamera::dump_color_frame(const std::string& filename)
{
		rs2::video_frame color = current_frameset.get_color_frame();
		stbi_write_png(filename.c_str(), color.get_width(), color.get_height(),
			color.get_bytes_per_pixel(), color.get_data(), color.get_stride_in_bytes());

}

MFCapture::MFCapture(const char *_configFilename)
:	numberOfPCsProduced(0),
	mergedPC_is_fresh(false),
	mergedPC_want_new(false)
{
	numberOfCapturersActive++;
	if (numberOfCapturersActive > 1) {
		std::cerr << "cwipc_realsense2: multiFrame: Warning: attempting to create capturer while one is already active." << std::endl;
	}
	if (_configFilename) {
		configFilename = _configFilename;
	}
	else {
		configFilename = "cameraconfig.xml";
	}
	// Create librealsense context for managing all connected RealSense devices
	auto devs = ctx.query_devices();
	const std::string platform_camera_name = "Platform Camera";
	// Determine how many realsense cameras (not platform cameras like webcams) are connected
	int camera_count = 0;
	for(auto dev: devs) {
		if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name) {
			camera_count++;
		}
	}
	//Sync messages to assign master and slave
#ifdef WITH_INTER_CAM_SYNC
	bool master_set = false;
#endif // WITH_INTER_CAM_SYNC
	//
	// Enumerate over all connected cameras, create their default MFCameraData structures
	// and set any hardware options (for example for sync).
	// We will create the actual MFCamera objects later, after we have read the configuration file.
	//
	for (auto dev : devs) {
#ifdef CWIPC_DEBUG
		std::cout << "MFCapture: looking at camera " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
#endif
		if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name) {
			// Found a realsense camera. Create a default data entry for it.
			MFCameraData cd;
			cd.serial = std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
			boost::shared_ptr<Eigen::Affine3d> default_trafo(new Eigen::Affine3d());
			default_trafo->setIdentity();
			cd.trafo = default_trafo;
			cd.cloud = new_cwipc_pcl_pointcloud();
			cd.background = { 0, 0, 0 };
			cd.cameraposition = { 0, 0, 0 };
			configuration.cameraData.push_back(cd);
#ifdef WITH_INTER_CAM_SYNC
			if (camera_count > 1) {
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

	if (camera_count == 0) {
		// no camera connected, so we'll use a generated pointcloud instead
		generatedPC = generate_pcl();
		std::cerr << "cwipc_realsense2: multiFrame: No cameras found, default production is a spinning generated pointcloud of " << generatedPC->size() << " data points\n";
	}
	else {
		// Read the configuration
		if (!mf_file2config(configFilename.c_str(), &configuration)) {

			// the configuration file did not fully match the current situation so we have to update the admin
			std::vector<std::string> serials;
			std::vector<MFCameraData> realcams;

			// collect serial numbers of all connected cameras
			for (auto dev : devs) {
				if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name) {
					serials.push_back(std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)));
				}
			}
			
			// collect all camera's in the config that are connected
			for (MFCameraData cd : configuration.cameraData) {
				if ((find(serials.begin(), serials.end(), cd.serial) != serials.end()))
					realcams.push_back(cd);
				else
					std::cerr << "cwipc_realsense2: multiFrame: Warning: camera " << cd.serial << " is not connected\n";
			}
			// Reduce the active configuration to cameras that are connected
			configuration.cameraData = realcams;
		}
	}
	// Now we have all the configuration information. Open the cameras.
	for (auto dev : devs) {
#ifdef CWIPC_DEBUG
		std::cout << "MFCapture: opening camera " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
#endif
		if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name) {
			// Found a realsense camera. Create a default data entry for it.
			MFCameraData& cd = get_camera_data(std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)));

			std::string camUsb(dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR));
			int camera_index = cameras.size();
			auto cam = new MFCamera(ctx, configuration, camera_index, cd, camUsb);
			cameras.push_back(cam);
		}
	}

	// Create an empty pointcloud just in case anyone calls get_mostRecentPointcloud() before one is generated.
	mergedPC = new_cwipc_pcl_pointcloud();

	// optionally set request for cwi_special_feature
	char* feature_request;
	feature_request = getenv("CWI_CAPTURE_FEATURE");
	if (feature_request != NULL)
		configuration.cwi_special_feature = feature_request;

	// find camerapositions
	for (int i = 0; i < configuration.cameraData.size(); i++) {
		cwipc_pcl_pointcloud pcptr(new_cwipc_pcl_pointcloud());
		cwipc_pcl_point pt;
		pt.x = 0;
		pt.y = 0;
		pt.z = 0;
		pcptr->push_back(pt);
		transformPointCloud(*pcptr, *pcptr, *configuration.cameraData[i].trafo);
		cwipc_pcl_point pnt = pcptr->points[0];
		configuration.cameraData[i].cameraposition.x = pnt.x;
		configuration.cameraData[i].cameraposition.y = pnt.y;
		configuration.cameraData[i].cameraposition.z = pnt.z;
	}


	// start the cameras
	starttime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	try {
		for (auto cam: cameras)
			cam->start(configuration);
	} catch(const rs2::error& e) {
		std::cerr << "cwipc_realsense2: exception while starting camera: " << e.get_failed_function() << ": " << e.what() << std::endl;
	}
	// start the per-camera capture threads
	for (auto cam: cameras)
		cam->start_capturer();
	// start our run thread (which will drive the capturers and merge the pointclouds)
	stopped = false;
	control_thread = new std::thread(&MFCapture::_control_thread_main, this);
}

MFCapture::~MFCapture() {
	uint64_t stopTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	if(!stopped) {
		// Make the control thread stop
		stopped = true;
		mergedPC_want_new = true;
		mergedPC_want_new_cv.notify_all();
		control_thread->join();
	}
	for (auto cam : cameras)
		cam->stop();
	std::cerr << "cwipc_realsense2: multiFrame: stopped all cameras\n";
	for (auto cam : cameras)
		delete cam;
	cameras.clear();
	std::cerr << "cwipc_realsense2: multiFrame: deleted all cameras\n";
	float deltaT = (stopTime - starttime) / 1000.0;
	std::cerr << "cwipc_realsense2: ran for " << deltaT << " seconds, produced " << numberOfPCsProduced << " pointclouds at " << numberOfPCsProduced / deltaT << " fps." << std::endl;
	numberOfCapturersActive--;
	assert(numberOfCapturersActive == 0);
}

// API function that triggers the capture and returns the merged pointcloud and timestamp
cwipc_pcl_pointcloud MFCapture::get_pointcloud(uint64_t *timestamp)
{
	*timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	_request_new_pointcloud();
	// Wait for a fresh mergedPC to become available.
	// Note we keep the return value while holding the lock, so we can start the next grab/process/merge cycle before returning.
	cwipc_pcl_pointcloud rv;
	{
		std::unique_lock<std::mutex> mylock(mergedPC_mutex);
		mergedPC_is_fresh_cv.wait(mylock, [this]{return mergedPC_is_fresh; });
		mergedPC_is_fresh = false;
		numberOfPCsProduced++;
		rv = mergedPC;
	}
	_request_new_pointcloud();
	return rv;
}

void MFCapture::_control_thread_main()
{
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "pointcloud processing: thread stopped" << std::endl;
#endif
	while(!stopped) {
		{
			std::unique_lock<std::mutex> mylock(mergedPC_mutex);
			mergedPC_want_new_cv.wait(mylock, [this]{ return mergedPC_want_new;});
		}
		if (stopped) break;
		if (cameras.size() > 0) {
			// Step one: grab frames from all cameras. This should happen as close together in time as possible,
			// because that gives use he biggest chance we have the same frame (or at most off-by-one) for each
			// camera.
			for(auto cam : cameras) {
				cam->capture_frameset();
			}
			// And get the best timestamp
			uint64_t timestamp = 0;
			for(auto cam: cameras) {
				uint64_t camts = cam->get_capture_timestamp();
				if (camts > timestamp) timestamp = camts;
			}
#ifdef WITH_DUMP_VIDEO_FRAMES
			// Step 2, if needed: dump image frames.
			if (configuration.cwi_special_feature == "dumpvideoframes") {
				for(auto cam : cameras) {
					std::stringstream png_file;
					png_file <<  "videoframe_" << timestamp - starttime << "_" << cam->camera_index << ".png";
					cam->dump_color_frame(png_file.str());
				}
			}
#endif // WITH_DUMP_VIDEO_FRAMES
			// Step 3: start processing frames to pointclouds, for each camera
			for(auto cam : cameras) {
				cam->create_pc_from_frames();
			}
			// Step 4: wait for frame processing to complete.
			for(auto cam : cameras) {
				cam->wait_for_pc();
			}
			// Step 5: merge views
			// Lock mergedPC while we are modifying it
			std::unique_lock<std::mutex> mylock(mergedPC_mutex);
			mergedPC = new_cwipc_pcl_pointcloud();
			merge_views();
			if (mergedPC->size() > 0) {
#ifdef CWIPC_DEBUG
				std::cerr << "cwipc_realsense2: multiFrame: capturer produced a merged cloud of " << mergedPC->size() << " points" << std::endl;
#endif
			} else {
#ifdef CWIPC_DEBUG
				std::cerr << "cwipc_realsense2: multiFrame: Warning: capturer got an empty pointcloud\n";
#endif
				// HACK to make sure the encoder does not get an empty pointcloud
				cwipc_pcl_point point;
				point.x = 1.0;
				point.y = 1.0;
				point.z = 1.0;
				point.rgb = 0.0;
				mergedPC->points.push_back(point);
			}
			// Signal that a new mergedPC is available. (Note that we acquired the mutex earlier)
			mergedPC_is_fresh = true;
			mergedPC_want_new = false;
			mergedPC_is_fresh_cv.notify_all();
		} else {	// return a spinning generated mathematical pointcloud
			static float angle;
			angle += 0.031415;
			Eigen::Affine3f transform = Eigen::Affine3f::Identity();
			transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()));
			std::unique_lock<std::mutex> mylock(mergedPC_mutex);
			// Lock mergedPC while we are modifying it
			mergedPC = new_cwipc_pcl_pointcloud();
			transformPointCloud(*generatedPC, *mergedPC, transform);
			// Signal that a new mergedPC is available
			mergedPC_is_fresh = true;
			mergedPC_want_new = false;
			mergedPC_is_fresh_cv.notify_all();
		}
	}
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "pointcloud processing: thread stopped" << std::endl;
#endif
}

// return the merged cloud 
cwipc_pcl_pointcloud MFCapture::get_mostRecentPointCloud()
{
	// This call doesn't need a fresh pointcloud (Jack thinks), but it does need one that is
	// consistent. So we lock, but don't wait on the condition.
	std::unique_lock<std::mutex> mylock(mergedPC_mutex);
	return mergedPC;
}

void MFCapture::_request_new_pointcloud()
{
	std::unique_lock<std::mutex> mylock(mergedPC_mutex);
	if (!mergedPC_want_new && !mergedPC_is_fresh) {
		mergedPC_want_new = true;
		mergedPC_want_new_cv.notify_all();
	}
}

void MFCapture::merge_views()
{
	cwipc_pcl_pointcloud aligned_cld(new_cwipc_pcl_pointcloud());
	mergedPC->clear();
	// Pre-allocate space in the merged pointcloud
	size_t nPoints = 0;
	for (MFCameraData cd : configuration.cameraData) {
		cwipc_pcl_pointcloud cam_cld = cd.cloud;
		nPoints += cam_cld->size();
	}
	mergedPC->reserve(nPoints);
	// Now transform and copy each pointcloud
	for (MFCameraData cd : configuration.cameraData) {
			cwipc_pcl_pointcloud cam_cld = cd.cloud;

		if (cam_cld->size() > 0) {
			transformPointCloud(*cam_cld, *aligned_cld, *cd.trafo);
			*mergedPC += *aligned_cld;
		}
	}

	if (configuration.cloud_resolution > 0) {
#ifdef CWIPC_DEBUG
		std::cerr << "cwipc_realsense2: multiFrame: Points before reduction: " << mergedPC->size() << std::endl;
#endif
		pcl::VoxelGrid<cwipc_pcl_point> grd;
		grd.setInputCloud(mergedPC);
		grd.setLeafSize(configuration.cloud_resolution, configuration.cloud_resolution, configuration.cloud_resolution);
		grd.setSaveLeafLayout(true);
		grd.filter(*mergedPC);

#ifdef CWIPC_DEBUG
		std::cerr << "cwipc_realsense2: multiFrame: Points after reduction: " << mergedPC->size() << std::endl;
#endif
	}
}

MFCameraData& MFCapture::get_camera_data(std::string serial) {
	for (int i = 0; i < configuration.cameraData.size(); i++)
		if (configuration.cameraData[i].serial == serial)
			return configuration.cameraData[i];
	std::cerr << "cwipc_realsense2: multiFrame: unknown camera " << serial << std::endl;
	abort();
}

MFCamera* MFCapture::get_camera(std::string serial) {
	for (auto cam : cameras)
		if (cam->serial == serial)
			return cam;
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

