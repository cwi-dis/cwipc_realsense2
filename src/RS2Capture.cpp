//
//  RS2Capture.cpp
//
//  Created by Fons Kuijk on 23-04-18
//

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

#if 0
rs2::context* RS2Capture::ctx_p = nullptr;
#endif

RS2Capture::RS2Capture() {
    type = "realsense";

    // First check that no other RS2Capture is active within this process (trying to catch programmer errors)
    numberOfCapturersActive++;

    if (numberOfCapturersActive > 1) {
        cwipc_rs2_log_warning("Attempting to create capturer while one is already active.");
    }
}

void RS2Capture::request_auxdata(bool _rgb, bool _depth, bool _timestamps) {
    configuration.auxData.want_auxdata_rgb = _rgb;
    configuration.auxData.want_auxdata_depth = _depth;
    configuration.auxData.want_image_timestamps = _timestamps;
}

bool RS2Capture::config_reload(const char *configFilename) {
    _unload_cameras();
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
    if (!_create_cameras()) {
        camera_count = 0;
        return false;
    }
    if (!_check_cameras_connected()) {
        camera_count = 0;
        return false;
    }

    _find_camera_positions();

    //
    // start the cameras
    //
    try {
        for (auto cam: cameras) {
            cam->start_camera();
        }
    } catch(const rs2::error& e) {
        cwipc_rs2_log_warning("Exception while starting camera: " + e.get_failed_function() + ": " + e.what());
        throw;
    }

    //
    // start the per-camera capture threads
    //
   for (auto cam: cameras) {
        cam->start_capturer();
    }
    
    for (auto cam: cameras) {
        cam->post_start_all_cameras();
    }
   
    //
    // start our run thread (which will drive the capturers and merge the pointclouds)
    //
    stopped = false;
    control_thread = new std::thread(&RS2Capture::_control_thread_main, this);
    _cwipc_setThreadName(control_thread, L"cwipc_realsense2::RS2Capture::control_thread");

    return true;
}

bool RS2Capture::_check_cameras_connected() {
    for (RS2CameraConfig& cd : configuration.all_camera_configs) {
        if (!cd.connected && !cd.disabled) {
            cwipc_rs2_log_warning("Camera " + cd.serial + " is not connected");
            return false;
        }
    }
    return true;
}

std::string RS2Capture::config_get() {
    bool match_only = false;
    // We get parameters like exposure here.
    // But framerate and width/height are gotten in the camera code.
    _refresh_camera_hardware_parameters();
    for(auto cam : cameras) {
        bool ok = cam->getHardwareParameters(configuration.hardware, match_only);
        if (!ok) {
            if (!match_only) {
                cwipc_rs2_log_warning("First camera does not have hardware configuration");
            } else {
                cwipc_rs2_log_warning("Not all cameras have the same hardware configuration");
            }
        }
        match_only = true;
    }
    return cwipc_rs2_config2string(&configuration);
}

void RS2Capture::_refresh_camera_hardware_parameters() {
    rs2::device_list devs = capturer_context.query_devices();
    rs2::device dev = *devs.begin();
    rs2::depth_sensor depth_sensor = dev.first<rs2::depth_sensor>();
    rs2::color_sensor color_sensor = dev.first<rs2::color_sensor>();

    bool auto_color_exposure = (bool)color_sensor.get_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE);
    int color_exposure = (int)color_sensor.get_option(RS2_OPTION_EXPOSURE);
    configuration.hardware.color_exposure = auto_color_exposure ? -color_exposure : color_exposure;  

    int color_gain = (int)color_sensor.get_option(RS2_OPTION_GAIN);
    configuration.hardware.color_gain = color_gain;
    
    bool auto_whitebalance = (bool)color_sensor.get_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE);
    int whitebalance = (int)color_sensor.get_option(RS2_OPTION_WHITE_BALANCE);
    configuration.hardware.whitebalance = auto_whitebalance ? -whitebalance : whitebalance;  

    configuration.hardware.backlight_compensation = (int)color_sensor.get_option(RS2_OPTION_BACKLIGHT_COMPENSATION);

    bool auto_depth_exposure = (bool)depth_sensor.get_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE);
    int depth_exposure = (int)depth_sensor.get_option(RS2_OPTION_EXPOSURE);
    configuration.hardware.depth_exposure = auto_depth_exposure ? -depth_exposure : depth_exposure;

    int depth_gain = (int)depth_sensor.get_option(RS2_OPTION_GAIN);
    configuration.hardware.depth_gain = depth_gain;

    configuration.hardware.laser_power = (int)depth_sensor.get_option(RS2_OPTION_LASER_POWER);

#ifdef xxxjack_disabled
    // It seems the visual preset is always returned as 0 (also seen in realsense-viewer)
    // so we don't try to get the value.
    configuration.hardware.visual_preset = (int)depth_sensor.get_option(RS2_OPTION_VISUAL_PRESET);
#endif
}


void RS2Capture::_setup_camera_hardware_parameters() {
    rs2::device_list devs = capturer_context.query_devices();
    for (auto dev : devs) {
        auto allSensors = dev.query_sensors();
        auto depth_sensor = dev.first<rs2::depth_sensor>();
        auto color_sensor = dev.first<rs2::color_sensor>();
        // We set visual preset first, because Jack is not sure whether it
        // contains values for some of the next settings.
        if (depth_sensor.supports(RS2_OPTION_VISUAL_PRESET)) {
            depth_sensor.set_option(
                RS2_OPTION_VISUAL_PRESET,
                (float)configuration.hardware.visual_preset
            );
        }
        // Options for color sensor
        if (configuration.hardware.color_gain >= 0) {
            assert(color_sensor.supports(RS2_OPTION_GAIN));
            color_sensor.set_option(RS2_OPTION_GAIN, configuration.hardware.color_gain);
        }
        if (configuration.hardware.color_exposure >= 0) {
            assert(color_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE));
            color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
            assert (color_sensor.supports(RS2_OPTION_EXPOSURE));
            color_sensor.set_option(RS2_OPTION_EXPOSURE, configuration.hardware.color_exposure);
        } else {
            if (color_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
                color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
            }
        }
        if (configuration.hardware.whitebalance >= 0) {
            if (color_sensor.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)) {
                color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 0);
            }

            if (color_sensor.supports(RS2_OPTION_WHITE_BALANCE)) {
                color_sensor.set_option(RS2_OPTION_WHITE_BALANCE, configuration.hardware.whitebalance);
            }
        } else {
            if (color_sensor.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)) {
                color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 1);
            }
        }
        if (configuration.hardware.backlight_compensation >= 0) {
            if (color_sensor.supports(RS2_OPTION_BACKLIGHT_COMPENSATION)) {
                color_sensor.set_option(RS2_OPTION_BACKLIGHT_COMPENSATION, configuration.hardware.backlight_compensation);
            }
        }
        // Options for depth sensor
         if (configuration.hardware.depth_gain >= 0) {
            assert(depth_sensor.supports(RS2_OPTION_GAIN));
            depth_sensor.set_option(RS2_OPTION_GAIN, configuration.hardware.depth_gain);
        }
        if (configuration.hardware.depth_exposure >= 0) {
            assert(depth_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE));
            depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
            assert(depth_sensor.supports(RS2_OPTION_EXPOSURE));
            depth_sensor.set_option(RS2_OPTION_EXPOSURE, configuration.hardware.depth_exposure);
        } else {
            if (depth_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
                depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
            }
        }
        if (depth_sensor.supports(RS2_OPTION_LASER_POWER) && configuration.hardware.laser_power >= 0) {
            depth_sensor.set_option(RS2_OPTION_LASER_POWER, configuration.hardware.laser_power);
        }

    
    }
}

void RS2Capture::_setup_camera_sync() {
    std::string master_serial = configuration.sync.sync_master_serial;
    if (master_serial == "") {
        // Disable sync
        for(auto sensor : capturer_context.query_all_sensors()) {
           if (sensor.supports(RS2_OPTION_INTER_CAM_SYNC_MODE)) {
                sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 0);
           }
        }
        return;
    }
    int nonmaster_sync_mode = configuration.sync.sync_mode;
    if (nonmaster_sync_mode == 0) {
        cwipc_rs2_log_warning("Sync_master_serial set, but no sync mode requested");
        return;
    }
    bool use_external_sync = master_serial == "external";
    bool master_found = use_external_sync;

    rs2::device_list devs = capturer_context.query_devices();
    bool is_first_camera = true;
    for (auto dev : devs) {
        std::string serial(std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)));
        bool is_master = serial == master_serial;
        for (auto sensor : dev.query_sensors()) {
            if (sensor.supports(RS2_OPTION_INTER_CAM_SYNC_MODE)) {
                    
                if (is_master) {
                    sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 1);
                    master_found = true;
                    if (!is_first_camera) {
                        cwipc_rs2_log_warning("Camera " + master_serial + " is not the first camera found. This may influence sync");
                    }
                } else {
                    sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, nonmaster_sync_mode);
                }
            }
        }
        is_first_camera = false;
    }
    if (!master_found) {
        cwipc_rs2_log_warning("Camera sync_master_serial=" + master_serial + " not found");
    }
#if 0
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
                    } else {
                        sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 2);
                    }
                }
            }

            if (!foundSensorSupportingSync) {
                cwipc_rs2_log_warning("Camera " + std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) + " does not support inter-camera-sync");
            }
        }
    }
#endif
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
    rs2::context temp_context;
    rs2::device_list devs = temp_context.query_devices();
    int camera_count = 0;

    for (rs2::device dev : devs) {
        if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name) {
            camera_count++;
        }
    }

    return camera_count;
}

bool RS2Capture::_apply_config(const char* configFilename) {
    // Clear out old configuration
    RS2CaptureConfig newConfiguration;
    newConfiguration.auxData = configuration.auxData;
    configuration = newConfiguration;

    //
    // Read the configuration. We do this only now because for historical reasons the configuration
    // reader is also the code that checks whether the configuration file contents match the actual
    // current hardware setup. To be fixed at some point.
    //
    if (configFilename == NULL || *configFilename == '\0') {
        // Empty config filename: use default cameraconfig.json.
        configFilename = "cameraconfig.json";
    }

    if (strcmp(configFilename, "auto") == 0) {
        // Special case 1: string "auto" means auto-configure all realsense cameras.
        return _apply_default_config();
    }

    if (configFilename[0] == '{') {
        // Special case 2: a string starting with { is considered a JSON literal
        return cwipc_rs2_jsonbuffer2config(configFilename, &configuration, type);
    }

    // Otherwise we check the extension. It can be .xml or .json.
    const char *extension = strrchr(configFilename, '.');
    if (extension != nullptr && strcmp(extension, ".xml") == 0) {
        return cwipc_rs2_xmlfile2config(configFilename, &configuration, type);
    }

    if (extension != nullptr && strcmp(extension, ".json") == 0) {
        return cwipc_rs2_jsonfile2config(configFilename, &configuration, type);
    }

    return false;
}

bool RS2Capture::_apply_default_config() {
    // Determine how many realsense cameras (not platform cameras like webcams) are connected
    const std::string platform_camera_name = "Platform Camera";
    rs2::device_list devs = capturer_context.query_devices();

    //
    // Enumerate over all connected cameras, create their default RS2CameraData structures
    // and set any hardware options (for example for sync).
    // We will create the actual RS2Camera objects later, after we have read the configuration file.
    //
    for (auto dev : devs) {
        if (dev.get_info(RS2_CAMERA_INFO_NAME) == platform_camera_name) {
            continue;
        }

#ifdef CWIPC_DEBUG
        if (configuration.debug) std::cerr << "cwipc_realsense2: looking at camera " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
#endif

        // Found a realsense camera. Create a default data entry for it.
        RS2CameraConfig cd;
        cd.serial = std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        pcl::shared_ptr<Eigen::Affine3d> default_trafo(new Eigen::Affine3d());
        default_trafo->setIdentity();
        cd.trafo = default_trafo;
        cd.cameraposition = { 0, 0, 0 };
        configuration.all_camera_configs.push_back(cd);
    }

    return true;

}

bool RS2Capture::_create_cameras() {
    rs2::device_list devs = capturer_context.query_devices();
    const std::string platform_camera_name = "Platform Camera";

    for (auto dev : devs) {
        if (dev.get_info(RS2_CAMERA_INFO_NAME) == platform_camera_name) {
            continue;
        }

#ifdef CWIPC_DEBUG
        if (configuration.debug) std::cerr << "cwipc_realsense2: opening camera " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
#endif

        // Found a realsense camera. Create a default data entry for it.
        std::string serial(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        
        RS2CameraConfig* cd = get_camera_config(serial);

        if (cd == nullptr) {
            cwipc_rs2_log_warning("Camera " + serial + " is connected but not configured");
            return false;
        }

        if (cd->type != "realsense") {
            cwipc_rs2_log_warning("Camera " + serial + " is type " + cd->type + " in stead of realsense");
            return false;
        }

        int camera_index = (int)cameras.size();

        if (cd->disabled) {
            // xxxnacho do we need to close the device, like the kinect case?
        } else {
            auto cam = new RS2Camera(capturer_context, configuration, camera_index);
            cameras.push_back(cam);
            cd->connected = true;
        }
    }

    return true;
}

RS2Capture::~RS2Capture() {
    _unload_cameras();
    numberOfCapturersActive--;
}

void RS2Capture::_unload_cameras() {
    uint64_t stopTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    // Stop all cameras
    for (auto cam : cameras) {
        cam->pre_stop_camera();
    }
    for (auto cam : cameras) {
        cam->stop_camera();
    }

    camera_count = 0;
    mergedPC_is_fresh = true;
    mergedPC_want_new = false;
    mergedPC_is_fresh_cv.notify_all();
    mergedPC_want_new = true;
    mergedPC_want_new_cv.notify_all();

    if (!stopped) {
        // Make the control thread stop. We set want_new to make it wake up (bit of a hack, really...)
        stopped = true;
        if (control_thread) {
            control_thread->join();
        }

        delete control_thread;
        control_thread = nullptr;
    }
#ifdef CWIPC_DEBUG
    if (configuration.debug) std::cerr << "cwipc_realsense2: stopped all cameras\n";
#endif

    // Delete all cameras (which will stop their threads as well)
    for (auto cam : cameras) {
        delete cam;
    }

    cameras.clear();
#ifdef CWIPC_DEBUG
    if (configuration.debug) std::cerr << "cwipc_realsense2: deleted all cameras\n";

    float deltaT = (stopTime - starttime) / 1000.0;
    if (configuration.debug) std::cerr << "cwipc_realsense2: ran for " << deltaT << " seconds, produced " << numberOfPCsProduced << " pointclouds at " << numberOfPCsProduced / deltaT << " fps." << std::endl;
#endif
}

// API function that triggers the capture and returns the merged pointcloud and timestamp
cwipc* RS2Capture::get_pointcloud() {
    if (camera_count == 0) {
        return nullptr;
    }

    _request_new_pointcloud();

    // Wait for a fresh mergedPC to become available.
    // Note we keep the return value while holding the lock, so we can start the next grab/process/merge cycle before returning.
    cwipc *rv;

    {
        std::unique_lock<std::mutex> mylock(mergedPC_mutex);

        mergedPC_is_fresh_cv.wait(mylock, [this] {
            return mergedPC_is_fresh;
        });

        mergedPC_is_fresh = false;
        rv = mergedPC;
    }

    _request_new_pointcloud();
    return rv;
}

float RS2Capture::get_pointSize() {
    if (camera_count == 0) {
        return 0;
    }

    float rv = 99999;
    for (auto cam : cameras) {
        if (cam->pointSize < rv) {
            rv = cam->pointSize;
        }
    }

    if (rv > 9999) {
        rv = 0;
    }

    return rv;
}

bool RS2Capture::map2d3d(int tile, int x_2d, int y_2d, int d_2d, float *out3d)
{
    for(auto cam : cameras) {
        if (tile == (1 << cam->camera_index)) {
            return cam->map2d3d(x_2d, y_2d, d_2d, out3d);
        }
    }
    return false;
}

bool RS2Capture::mapcolordepth(int tile, int u, int v, int *out2d)
{
    for(auto cam : cameras) {
        if (tile == (1 << cam->camera_index)) {
            return cam->mapcolordepth(u, v, out2d);
        }
    }
    return false;
}

bool RS2Capture::pointcloud_available(bool wait)
{
    if (camera_count == 0) {
        return false;
    }

    _request_new_pointcloud();

    std::this_thread::yield();
    std::unique_lock<std::mutex> mylock(mergedPC_mutex);

    auto duration = std::chrono::seconds(wait?1:0);
    mergedPC_is_fresh_cv.wait_for(mylock, duration, [this]{
        return mergedPC_is_fresh;
    });

    return mergedPC_is_fresh;
}

void RS2Capture::_initial_camera_synchronization() {

}

void RS2Capture::_control_thread_main() {
#ifdef CWIPC_DEBUG_THREAD
    if (configuration.debug) std::cerr << "cwipc_realsense2: processing thread started" << std::endl;
#endif
    _initial_camera_synchronization();
    while(!stopped) {
        {
            std::unique_lock<std::mutex> mylock(mergedPC_mutex);
            mergedPC_want_new_cv.wait(mylock, [this]{
              return mergedPC_want_new;
            });
        }

        if (stopped) {
            break;
        }

        assert (cameras.size() > 0);

        // Step one: grab frames from all cameras. This should happen as close together in time as possible,
        // because that gives use he biggest chance we have the same frame (or at most off-by-one) for each
        // camera.
        uint64_t first_timestamp = 0;
        for(auto cam : cameras) {
            uint64_t this_cam_timestamp = cam->wait_for_captured_frameset(first_timestamp);
            if (first_timestamp == 0) {
                first_timestamp = this_cam_timestamp;
            }
        }

        // And get the best timestamp
        uint64_t timestamp = first_timestamp;
        if (configuration.new_timestamps) {
            timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        } else if (timestamp == 0) {
            std::cerr << "cwipc_realsense2: Warning: using system clock timestamp" << std::endl;
            timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        }
        if (configuration.debug) std::cerr << "RS2Capture: creating pc with ts=" << timestamp << std::endl;
        // step 2 : create pointcloud, and save rgb/depth frames if wanted
        if (mergedPC && mergedPC_is_fresh) {
            mergedPC->free();
            mergedPC = nullptr;
        }

        cwipc_pcl_pointcloud pcl_pointcloud = new_cwipc_pcl_pointcloud();
        mergedPC = cwipc_from_pcl(pcl_pointcloud, timestamp, NULL, CWIPC_API_VERSION);

        for (auto cam : cameras) {
            cam->save_frameset_auxdata(mergedPC);
        }
    
        // Step 3: start processing frames to pointclouds, for each camera
        for(auto cam : cameras) {
            cam->create_pc_from_frameset();
        }

        // Lock mergedPC already while we are waiting for the per-camera
        // processing threads. This so the main thread doesn't go off and do
        // useless things if it is calling available(true).
        std::unique_lock<std::mutex> mylock(mergedPC_mutex);

        // Step 4: wait for frame processing to complete.
        for(auto cam : cameras) {
            cam->wait_for_pc_created();
        }

        // Step 5: merge views
        merge_camera_pointclouds();

        if (mergedPC->access_pcl_pointcloud()->size() > 0) {
#ifdef CWIPC_DEBUG
            if (configuration.debug) std::cerr << "cwipc_realsense2: capturer produced a merged cloud of " << mergedPC->size() << " points" << std::endl;
#endif
        } else {
#ifdef CWIPC_DEBUG
            if (configuration.debug) std::cerr << "cwipc_realsense2: Warning: capturer got an empty pointcloud\n";
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
    if (configuration.debug) std::cerr << "cwipc_realsense2: processing thread stopped" << std::endl;
#endif
}

void RS2Capture::_request_new_pointcloud() {
    std::unique_lock<std::mutex> mylock(mergedPC_mutex);

    if (!mergedPC_want_new && !mergedPC_is_fresh) {
        mergedPC_want_new = true;
        mergedPC_want_new_cv.notify_all();
    }
}

void RS2Capture::merge_camera_pointclouds() {
    cwipc_pcl_pointcloud aligned_cld(mergedPC->access_pcl_pointcloud());

    // Pre-allocate space in the merged pointcloud
    size_t nPoints = 0;

    for (auto cam : cameras) {
        cwipc_pcl_pointcloud cam_cld = cam->access_current_pcl_pointcloud();

        if (cam_cld == 0) {
            cwipc_rs2_log_warning("Camera " + cam->serial + " has NULL cloud");
            continue;
        }
        nPoints += cam_cld->size();
    }

    aligned_cld->reserve(nPoints);

    // Now merge all pointclouds
    for (auto cam : cameras) {
        cwipc_pcl_pointcloud cam_cld = cam->access_current_pcl_pointcloud();

        if (cam_cld == NULL) {
            continue;
        }

        *aligned_cld += *cam_cld;
    }

    // No need to merge aux_data: already inserted into mergedPC by each camera
}

RS2CameraConfig* RS2Capture::get_camera_config(std::string serial) {
    for (int i = 0; i < configuration.all_camera_configs.size(); i++) {
        if (configuration.all_camera_configs[i].serial == serial) {
            return &configuration.all_camera_configs[i];
        }
    }

    cwipc_rs2_log_warning("Unknown camera " + serial);
    return nullptr;
}
