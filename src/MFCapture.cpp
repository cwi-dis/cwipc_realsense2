//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//

// Define to try and use hardware sync to synchronize multiple cameras
#define WITH_INTER_CAM_SYNC

// Define to get (a little) debug prints
#undef CWIPC_DEBUG
#undef CWIPC_DEBUG_THREAD

// This is the dll source, so define external symbols as dllexport on windows.

#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#define CWIPC_DLL_ENTRY __declspec(dllexport)
#endif

#include <chrono>
#if 0
#include <cstdint>
#include "cwipc_realsense2/multiFrame.hpp"
#include "cwipc_realsense2/api.h"
#include "cwipc_realsense2/utils.h"
#endif

#include "cwipc_realsense2/defs.h"
#include "cwipc_realsense2/utils.h"
#include "cwipc_realsense2/MFCapture.hpp"
#include "cwipc_realsense2/MFCamera.hpp"

// Static variable used to print a warning message when we re-create an MFCapture
// if there is another one open.
static int numberOfCapturersActive = 0;

MFCapture::MFCapture(int dummy)
:	numberOfPCsProduced(0),
	mergedPC_is_fresh(false),
	mergedPC_want_new(false)
{
	numberOfCapturersActive++;
	mergedPC = new_cwipc_pcl_pointcloud();
	starttime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

MFCapture::MFCapture(const char *configFilename)
:	numberOfPCsProduced(0),
    no_cameras(false),
	mergedPC_is_fresh(false),
	mergedPC_want_new(false)
{
	// First check that no other MFCapture is active within this process (trying to catch programmer errors)
	numberOfCapturersActive++;
	if (numberOfCapturersActive > 1) {
		mf_log_warning("multiFrame: Warning: attempting to create capturer while one is already active.");
	}

	// Determine how many realsense cameras (not platform cameras like webcams) are connected
	const std::string platform_camera_name = "Platform Camera";
	rs2::device_list devs = ctx.query_devices();
	int camera_count = 0;
	for(auto dev: devs) {
		if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name) {
			camera_count++;
		}
	}
	if (camera_count == 0) {
		// no camera connected, so we'll return nothing
        no_cameras = true;
		return;
	}
	//
	// Enumerate over all connected cameras, create their default MFCameraData structures
	// and set any hardware options (for example for sync).
	// We will create the actual MFCamera objects later, after we have read the configuration file.
	//
	for (auto dev : devs) {
		if (dev.get_info(RS2_CAMERA_INFO_NAME) == platform_camera_name) continue;
		// Found a realsense camera. Create a default data entry for it.
#ifdef CWIPC_DEBUG
		std::cout << "MFCapture: looking at camera " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
#endif
		MFCameraData cd;
		cd.serial = std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
		boost::shared_ptr<Eigen::Affine3d> default_trafo(new Eigen::Affine3d());
		default_trafo->setIdentity();
		cd.trafo = default_trafo;
		cd.intrinsicTrafo = default_trafo;
		cd.cloud = new_cwipc_pcl_pointcloud();
		cd.background = { 0, 0, 0 };
		cd.cameraposition = { 0, 0, 0 };
		configuration.cameraData.push_back(cd);
	}

	//
	// Read the configuration. We do this only now because for historical reasons the configuration
	// reader is also the code that checks whether the configuration file contents match the actual
	// current hardware setup. To be fixed at some point.
	//
	if (configFilename == NULL) {
		configFilename = "cameraconfig.xml";
	}
	if (!mf_file2config(configFilename, &configuration)) {

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
				mf_log_warning("multiFrame: Warning: camera " + cd.serial + " is not connected");
		}
		// Reduce the active configuration to cameras that are connected
		configuration.cameraData = realcams;
	}
    // Set various camera hardware parameters (white balance and such)
    for (auto dev : devs) {
        auto allSensors = dev.query_sensors();
        for (auto sensor : allSensors) {
            // Options for color sensor (but may work inadvertantly on dept sensors too?)
            if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
            if (sensor.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE))
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 0);
            if (sensor.supports(RS2_OPTION_BACKLIGHT_COMPENSATION))
                sensor.set_option(RS2_OPTION_BACKLIGHT_COMPENSATION, 0);
            // Options for depth sensor
            if (sensor.supports(RS2_OPTION_LASER_POWER))
                sensor.set_option(RS2_OPTION_LASER_POWER, 360);
            // xxxjack note: the document at <https://github.com/IntelRealSense/librealsense/wiki/D400-Series-Visual-Presets>
            // suggests that this may depend on using 1280x720@30 with decimation=3. Need to check.
            if (sensor.supports(RS2_OPTION_VISUAL_PRESET)) {
                sensor.set_option(RS2_OPTION_VISUAL_PRESET,
					configuration.density
						? RS2_RS400_VISUAL_PRESET_HIGH_DENSITY
						: RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY
				);
			}
        }
    }
	//
	// Set sync mode, if needed
	//
#ifdef WITH_INTER_CAM_SYNC
	bool master_set = false;
	for(auto dev : devs) {
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
                mf_log_warning(std::string("multiFrame: Warning: camera ") + dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) + " does not support inter-camera-sync");
			}
		}
	}
#endif // WITH_INTER_CAM_SYNC
	// Now we have all the configuration information. Open the cameras.
	_create_cameras(devs);

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

	//
	// start the cameras
	//
	try {
		for (auto cam: cameras)
			cam->start();
	} catch(const rs2::error& e) {
		mf_log_warning("exception while starting camera: " + e.get_failed_function() + ": " + e.what());
		throw;
	}
	starttime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	//
	// start the per-camera capture threads
	//
	for (auto cam: cameras)
		cam->start_capturer();
	//
	// start our run thread (which will drive the capturers and merge the pointclouds)
	//
	stopped = false;
	control_thread = new std::thread(&MFCapture::_control_thread_main, this);
}

void MFCapture::_create_cameras(rs2::device_list devs) {
	const std::string platform_camera_name = "Platform Camera";
	for (auto dev : devs) {
		if (dev.get_info(RS2_CAMERA_INFO_NAME) == platform_camera_name) continue;
#ifdef CWIPC_DEBUG
		std::cout << "MFCapture: opening camera " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
#endif
		// Found a realsense camera. Create a default data entry for it.
		std::string serial(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
		std::string camUsb(dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR));

		MFCameraData& cd = get_camera_data(serial);
		int camera_index = cameras.size();
		auto cam = new MFCamera(ctx, configuration, camera_index, cd, camUsb);
		cameras.push_back(cam);
	}
}

MFCapture::~MFCapture() {
    if (no_cameras) return;
	uint64_t stopTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	// Stop all cameras
	for (auto cam : cameras)
		cam->stop();
	if(!stopped) {
		// Make the control thread stop. We set want_new to make it wake up (bit of a hack, really...)
		stopped = true;
		mergedPC_want_new = true;
		mergedPC_want_new_cv.notify_all();
		control_thread->join();
	}
	std::cerr << "cwipc_realsense2: multiFrame: stopped all cameras\n";
	// Delete all cameras (which will stop their threads as well)
	for (auto cam : cameras)
		delete cam;
	cameras.clear();
	std::cerr << "cwipc_realsense2: multiFrame: deleted all cameras\n";
	// Print some minimal statistics of this run
	float deltaT = (stopTime - starttime) / 1000.0;
	std::cerr << "cwipc_realsense2: ran for " << deltaT << " seconds, produced " << numberOfPCsProduced << " pointclouds at " << numberOfPCsProduced / deltaT << " fps." << std::endl;
	numberOfCapturersActive--;
}

// API function that triggers the capture and returns the merged pointcloud and timestamp
cwipc_pcl_pointcloud MFCapture::get_pointcloud(uint64_t *timestamp)
{
    if (no_cameras) return nullptr;
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

float MFCapture::get_pointSize()
{
    if (no_cameras) return 0;
	float rv = 99999;
	for (auto cam : cameras) {
		if (cam->pointSize < rv) rv = cam->pointSize;
	}
	if (rv > 9999) rv = 0;
	return rv;
}

bool MFCapture::pointcloud_available(bool wait)
{
    if (no_cameras) return false;
	_request_new_pointcloud();
	std::this_thread::yield();
	std::unique_lock<std::mutex> mylock(mergedPC_mutex);
	auto duration = std::chrono::seconds(wait?1:0);
	mergedPC_is_fresh_cv.wait_for(mylock, duration, [this]{return mergedPC_is_fresh; });
	return mergedPC_is_fresh;
}

void MFCapture::_control_thread_main()
{
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "MFCapture: processing thread started" << std::endl;
#endif
	while(!stopped) {
		{
			std::unique_lock<std::mutex> mylock(mergedPC_mutex);
			mergedPC_want_new_cv.wait(mylock, [this]{ return mergedPC_want_new;});
		}
		if (stopped) break;
        assert (cameras.size() > 0);
        // Step one: grab frames from all cameras. This should happen as close together in time as possible,
        // because that gives use he biggest chance we have the same frame (or at most off-by-one) for each
        // camera.
        for(auto cam : cameras) {
            if (!cam->capture_frameset()) continue;
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
        // Lock mergedPC already while we are waiting for the per-camera
        // processing threads. This so the main thread doesn't go off and do
        // useless things if it is calling available(true).
        std::unique_lock<std::mutex> mylock(mergedPC_mutex);
        // Step 4: wait for frame processing to complete.
        for(auto cam : cameras) {
            cam->wait_for_pc();
        }
        // Step 5: merge views
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
	}
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "MFCapture: processing thread stopped" << std::endl;
#endif
}

// return the merged cloud 
cwipc_pcl_pointcloud MFCapture::get_mostRecentPointCloud()
{
    if (no_cameras) return nullptr;
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
	// Now merge all pointclouds
	for (MFCameraData cd : configuration.cameraData) {
		cwipc_pcl_pointcloud cam_cld = cd.cloud;
		*mergedPC += *cam_cld;
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
	mf_log_warning("cwipc_realsense2: multiFrame: unknown camera " + serial);
	abort();
}

MFCamera* MFCapture::get_camera(std::string serial) {
	for (auto cam : cameras)
		if (cam->serial == serial)
			return cam;
	return NULL;
}
