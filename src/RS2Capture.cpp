//
//  RS2Capture.cpp
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

#include "cwipc_realsense2/private/RS2Config.hpp"
#include "cwipc_realsense2/private/RS2Capture.hpp"
#include "cwipc_realsense2/private/RS2Camera.hpp"

// Static variable used to print a warning message when we re-create an RS2Capture
// if there is another one open.
static int numberOfCapturersActive = 0;

rs2::context RS2Capture::ctx;

RS2Capture::RS2Capture()
{
	// First check that no other RS2Capture is active within this process (trying to catch programmer errors)
	numberOfCapturersActive++;
	if (numberOfCapturersActive > 1) {
		cwipc_rs2_log_warning("Attempting to create capturer while one is already active.");
	}
}

bool RS2Capture::config_reload(const char *configFilename) {
    camera_count = 0;
    if (!_apply_config(configFilename)) {
        return false;
    }
    camera_count = (int)configuration.all_camera_configs.size();
    if (camera_count == 0) {
        return false;
    }
    // Set various camera hardware parameters (white balance and such)
    _setup_camera_hardware_parameters();

    //
    // Set sync mode, if needed
    //
    _setup_camera_sync();

    // Now we have all the configuration information. Open the cameras.
    _create_cameras();


    _find_camera_positions();



    //
    // start the cameras
    //
    try {
        for (auto cam: cameras)
            cam->start();
    } catch(const rs2::error& e) {
        cwipc_rs2_log_warning("Exception while starting camera: " + e.get_failed_function() + ": " + e.what());
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
    control_thread = new std::thread(&RS2Capture::_control_thread_main, this);
    _cwipc_setThreadName(control_thread, L"cwipc_realsense2::RS2Capture::control_thread");
	return true;
}

std::string RS2Capture::config_get() {
    return "";
}

void RS2Capture::_setup_camera_hardware_parameters() {
	rs2::device_list devs = ctx.query_devices();
	for (auto dev : devs) {
		auto allSensors = dev.query_sensors();
		for (auto sensor : allSensors) {
			// Options for color sensor (but may work inadvertantly on dept sensors too?)
			if (configuration.exposure >= 0) {
				if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
					sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
				if (sensor.supports(RS2_OPTION_EXPOSURE))
					sensor.set_option(RS2_OPTION_EXPOSURE, configuration.exposure);
			}
			else {
				if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
					sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
			}
			if (configuration.whitebalance >= 0) {
				if (sensor.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE))
					sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 0);
				if (sensor.supports(RS2_OPTION_WHITE_BALANCE))
					sensor.set_option(RS2_OPTION_WHITE_BALANCE, configuration.whitebalance);
			}
			else {
				if (sensor.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE))
					sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 1);
			}
			if (configuration.backlight_compensation >= 0) {
				if (sensor.supports(RS2_OPTION_BACKLIGHT_COMPENSATION))
					sensor.set_option(RS2_OPTION_BACKLIGHT_COMPENSATION, configuration.backlight_compensation);
			}
			// Options for depth sensor
			if (sensor.supports(RS2_OPTION_LASER_POWER) && configuration.laser_power >= 0) {
				sensor.set_option(RS2_OPTION_LASER_POWER, configuration.laser_power);
			}
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
}

void RS2Capture::_setup_camera_sync() {
#ifdef WITH_INTER_CAM_SYNC
	rs2::device_list devs = ctx.query_devices();
	bool master_set = false;
	for (auto dev : devs) {
		if (camera_count > 1) {
			auto allSensors = dev.query_sensors();
			bool foundSensorSupportingSync = false;
			for (auto sensor : allSensors) {
				if (sensor.supports(RS2_OPTION_INTER_CAM_SYNC_MODE)) {
					foundSensorSupportingSync = true;
					if (!master_set) {
						sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 1);
						master_set = true;
					}
					else {
						sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 2);
					}
				}
			}
			if (!foundSensorSupportingSync) {
				cwipc_rs2_log_warning("Camera " + std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) + " does not support inter-camera-sync");
			}
		}
	}
#endif // WITH_INTER_CAM_SYNC
}

void RS2Capture::_find_camera_positions() {
	// find camerapositions
	for (int i = 0; i < configuration.all_camera_configs.size(); i++) {
		cwipc_pcl_pointcloud pcptr(new_cwipc_pcl_pointcloud());
		cwipc_pcl_point pt;
		pt.x = 0;
		pt.y = 0;
		pt.z = 0;
		pcptr->push_back(pt);
		transformPointCloud(*pcptr, *pcptr, *configuration.all_camera_configs[i].trafo);
		cwipc_pcl_point pnt = pcptr->points[0];
		configuration.all_camera_configs[i].cameraposition.x = pnt.x;
		configuration.all_camera_configs[i].cameraposition.y = pnt.y;
		configuration.all_camera_configs[i].cameraposition.z = pnt.z;
	}
}

int RS2Capture::count_devices() {
	// Determine how many realsense cameras (not platform cameras like webcams) are connected
	const std::string platform_camera_name = "Platform Camera";
	rs2::device_list devs = ctx.query_devices();
	int camera_count = 0;
	for (auto dev : devs) {
		if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name) {
			camera_count++;
		}
	}
	return camera_count;

}

bool RS2Capture::_apply_config(const char* configFilename) {
    // Clear out old configuration
    RS2CaptureConfig empty;
    configuration = empty;
    
	//
	// Read the configuration. We do this only now because for historical reasons the configuration
	// reader is also the code that checks whether the configuration file contents match the actual
	// current hardware setup. To be fixed at some point.
	//
	if (configFilename == NULL || *configFilename == '\0') {
        // Empty config filename: use default cameraconfig.xml.
		configFilename = "cameraconfig.xml";
	}
    if (strcmp(configFilename, "auto") == 0) {
        // Special case 1: string "auto" means auto-configure all realsense cameras.
        return _apply_default_config();
    }
    if (configFilename[0] == '{') {
        // Special case 2: a string starting with { is considered a JSON literal
        return cwipc_rs2_jsonbuffer2config(configFilename, &configuration);
    }
    // Otherwise we check the extension. It can be .xml or .json.
    const char *extension = strrchr(configFilename, '.');
    if (strcmp(extension, ".xml") == 0) {
        return cwipc_rs2_xmlfile2config(configFilename, &configuration);
    }
    if (strcmp(extension, ".json") == 0) {
        return cwipc_rs2_jsonfile2config(configFilename, &configuration);
    }
    return false;
}

bool RS2Capture::_apply_default_config() {
	// Determine how many realsense cameras (not platform cameras like webcams) are connected
	const std::string platform_camera_name = "Platform Camera";
	rs2::device_list devs = ctx.query_devices();
	
	//
	// Enumerate over all connected cameras, create their default RS2CameraData structures
	// and set any hardware options (for example for sync).
	// We will create the actual RS2Camera objects later, after we have read the configuration file.
	//
	for (auto dev : devs) {
		if (dev.get_info(RS2_CAMERA_INFO_NAME) == platform_camera_name) continue;
		// Found a realsense camera. Create a default data entry for it.
#ifdef CWIPC_DEBUG
		std::cout << "cwipc_realsense2: looking at camera " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
#endif
		RS2CameraConfig cd;
		cd.serial = std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
		pcl::shared_ptr<Eigen::Affine3d> default_trafo(new Eigen::Affine3d());
		default_trafo->setIdentity();
		cd.trafo = default_trafo;
        pcl::shared_ptr<Eigen::Affine3d> default_intrinsic_trafo(new Eigen::Affine3d());
        default_intrinsic_trafo->setIdentity();
        cd.intrinsicTrafo = default_intrinsic_trafo;
		cd.cameraposition = { 0, 0, 0 };
		configuration.all_camera_configs.push_back(cd);
	}
    return true;
#ifdef xxxjack_old

	// the configuration file did not fully match the current situation so we have to update the admin
	std::vector<std::string> serials;
	std::vector<RS2CameraData> realcams;

	// collect serial numbers of all connected cameras
	for (auto dev : devs) {
		if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name) {
			serials.push_back(std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)));
		}
	}

	// collect all camera's in the config that are connected
	for (RS2CameraData cd : configuration.all_camera_configs) {
		if ((find(serials.begin(), serials.end(), cd.serial) != serials.end())) {
			if (!cd.disabled)
				realcams.push_back(cd);
		}
		else {
			cwipc_rs2_log_warning("Camera " + cd.serial + " is not connected");
		}
	}
	// Reduce the active configuration to cameras that are connected
	configuration.all_camera_configs = realcams;
#endif

}

void RS2Capture::_create_cameras() {
	rs2::device_list devs = ctx.query_devices();
	const std::string platform_camera_name = "Platform Camera";
	for (auto dev : devs) {
		if (dev.get_info(RS2_CAMERA_INFO_NAME) == platform_camera_name) continue;
#ifdef CWIPC_DEBUG
		std::cout << "cwipc_realsense2: opening camera " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
#endif
		// Found a realsense camera. Create a default data entry for it.
		std::string serial(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
		std::string camUsb(dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR));

		RS2CameraConfig* cd = get_camera_config(serial);
		if (cd == nullptr) {
			cwipc_rs2_log_warning("Camera " + serial + " is not connected");
			continue;
		}
        if (cd->type != "realsense") {
            cwipc_rs2_log_warning("Camera " + serial + " is type " + cd->type + " in stead of realsense");
			continue;
        }
		int camera_index = (int)cameras.size();
		if (cd->disabled) {
			// xxxnacho do we need to close the device, like the kinect case?
		}else{
			auto cam = new RS2Camera(ctx, configuration, camera_index, *cd, camUsb);
			cameras.push_back(cam);
		}
	}
}

RS2Capture::~RS2Capture() {
    if (camera_count != 0) {
        _unload_cameras();
    }
    numberOfCapturersActive--;
}

void RS2Capture::_unload_cameras() {
    uint64_t stopTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    if (camera_count == 0) return;
    // Stop all cameras
    for (auto cam : cameras)
        cam->stop();
    camera_count = 0;
    mergedPC_is_fresh = true;
    mergedPC_want_new = false;
    mergedPC_is_fresh_cv.notify_all();
    mergedPC_want_new = true;
    mergedPC_want_new_cv.notify_all();
    if(!stopped) {
        // Make the control thread stop. We set want_new to make it wake up (bit of a hack, really...)
        stopped = true;
          control_thread->join();
    }
    std::cerr << "cwipc_realsense2: stopped all cameras\n";
    // Delete all cameras (which will stop their threads as well)
    for (auto cam : cameras)
        delete cam;
    cameras.clear();
    std::cerr << "cwipc_realsense2: deleted all cameras\n";
    // Print some minimal statistics of this run
    float deltaT = (stopTime - starttime) / 1000.0;
    std::cerr << "cwipc_realsense2: ran for " << deltaT << " seconds, produced " << numberOfPCsProduced << " pointclouds at " << numberOfPCsProduced / deltaT << " fps." << std::endl;
}

// API function that triggers the capture and returns the merged pointcloud and timestamp
cwipc* RS2Capture::get_pointcloud()
{
    if (camera_count == 0) return nullptr;
	_request_new_pointcloud();
	// Wait for a fresh mergedPC to become available.
	// Note we keep the return value while holding the lock, so we can start the next grab/process/merge cycle before returning.
	cwipc* rv;
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

float RS2Capture::get_pointSize()
{
    if (camera_count == 0) return 0;
	float rv = 99999;
	for (auto cam : cameras) {
		if (cam->pointSize < rv) rv = cam->pointSize;
	}
	if (rv > 9999) rv = 0;
	return rv;
}

bool RS2Capture::pointcloud_available(bool wait)
{
    if (camera_count == 0) return false;
	_request_new_pointcloud();
	std::this_thread::yield();
	std::unique_lock<std::mutex> mylock(mergedPC_mutex);
	auto duration = std::chrono::seconds(wait?1:0);
	mergedPC_is_fresh_cv.wait_for(mylock, duration, [this]{return mergedPC_is_fresh; });
	return mergedPC_is_fresh;
}

void RS2Capture::_control_thread_main()
{
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "cwipc_realsense2: processing thread started" << std::endl;
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
        if (timestamp == 0) {
            timestamp =  std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        }

        // step 2 : create pointcloud, and save rgb/depth frames if wanted
        if (mergedPC && mergedPC_is_fresh) {
            mergedPC->free();
            mergedPC = nullptr;
        }
        cwipc_pcl_pointcloud pcl_pointcloud = new_cwipc_pcl_pointcloud();
        mergedPC = cwipc_from_pcl(pcl_pointcloud, timestamp, NULL, CWIPC_API_VERSION);

        if (want_auxdata_rgb || want_auxdata_depth) {
            for (auto cam : cameras) {
                cam->save_auxdata(mergedPC, want_auxdata_rgb, want_auxdata_depth);
            }
        }
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
        merge_views();
        if (mergedPC->access_pcl_pointcloud()->size() > 0) {
#ifdef CWIPC_DEBUG
            std::cerr << "cwipc_realsense2: capturer produced a merged cloud of " << mergedPC->size() << " points" << std::endl;
#endif
        } else {
#ifdef CWIPC_DEBUG
            std::cerr << "cwipc_realsense2: Warning: capturer got an empty pointcloud\n";
#endif
#if 0
            // HACK to make sure the encoder does not get an empty pointcloud
            cwipc_pcl_point point;
            point.x = 1.0;
            point.y = 1.0;
            point.z = 1.0;
            point.rgb = 0.0;
            mergedPC->points.push_back(point);
#endif
        }
        // Signal that a new mergedPC is available. (Note that we acquired the mutex earlier)
        mergedPC_is_fresh = true;
        mergedPC_want_new = false;
        mergedPC_is_fresh_cv.notify_all();
	}
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "cwipc_realsense2: processing thread stopped" << std::endl;
#endif
}

// return the merged cloud 
cwipc* RS2Capture::get_mostRecentPointCloud()
{
    if (camera_count == 0) return nullptr;
	// This call doesn't need a fresh pointcloud (Jack thinks), but it does need one that is
	// consistent. So we lock, but don't wait on the condition.
	std::unique_lock<std::mutex> mylock(mergedPC_mutex);
	return mergedPC;
}

void RS2Capture::_request_new_pointcloud()
{
	std::unique_lock<std::mutex> mylock(mergedPC_mutex);
	if (!mergedPC_want_new && !mergedPC_is_fresh) {
		mergedPC_want_new = true;
		mergedPC_want_new_cv.notify_all();
	}
}

void RS2Capture::merge_views()
{
	cwipc_pcl_pointcloud aligned_cld(mergedPC->access_pcl_pointcloud());
	// Pre-allocate space in the merged pointcloud
	size_t nPoints = 0;
	for (auto cam : cameras) {
		cwipc_pcl_pointcloud cam_cld = cam->get_current_pointcloud();
        if (cam_cld == 0) {
            cwipc_rs2_log_warning("Camera " + cam->serial + " has NULL cloud");
            continue;
        }
		nPoints += cam_cld->size();
	}
	aligned_cld->reserve(nPoints);
	// Now merge all pointclouds
	for (auto cam : cameras) {
		cwipc_pcl_pointcloud cam_cld = cam->get_current_pointcloud();
        if (cam_cld == NULL) continue;
		*aligned_cld += *cam_cld;
	}
    // No need to merge aux_data: already inserted into mergedPC by each camera
}

RS2CameraConfig* RS2Capture::get_camera_config(std::string serial) {
	for (int i = 0; i < configuration.all_camera_configs.size(); i++)
		if (configuration.all_camera_configs[i].serial == serial)
			return &configuration.all_camera_configs[i];
	cwipc_rs2_log_warning("Unknown camera " + serial);
	return nullptr;
}

RS2Camera* RS2Capture::get_camera(std::string serial) {
	for (auto cam : cameras)
		if (cam->serial == serial)
			return cam;
	return NULL;
}
