#ifndef cwipc_realsense_RS2BaseCapture_hpp
#define cwipc_realsense_RS2BaseCapture_hpp
#pragma once


#include <thread>
#include <mutex>
#include <condition_variable>

#include <librealsense2/rs.hpp>

#include "cwipc_util/capturers.hpp"

#include "RS2Config.hpp"


/** Base class for capturers that use the librealsense API. 
 * 
 * For librealsense actually most of the implementation is in this class,
 * because a playback device has the same API as a live camera.
 * 
 * Subclasses need to implement factory() and count_devices().
*/
template <class Type_our_camera, class Type_our_camera_config>
class RS2BaseCapture : public CwipcBaseCapture {
public:
    /// Subclasses need to implement static factory().
    /// Subclasses need to implement static count_devices().

    using CwipcBaseCapture::CwipcBaseCapture;
    virtual ~RS2BaseCapture()  {
        _unload_cameras();
    }

    int get_camera_count() override final { 
        return cameras.size(); 
    }

    bool is_valid() override final {
        return cameras.size() > 0; 
    }

    virtual bool config_reload_and_start_capturing(const char *configFilename) override final {
        _unload_cameras();

        if (!_apply_config(configFilename)) {
            return false;
        }

        auto camera_config_count = configuration.all_camera_configs.size();
        if (camera_config_count == 0) {
            return false;
        }

        // Set various camera hardware parameters (white balance and such)
        _init_hardware_for_all_cameras();

        //
        // Set sync mode, if needed
        //
        _setup_inter_camera_sync();

        // Now we have all the configuration information. Open the cameras.
        // This will _not_ include the disabled cameras, so after this we should
        // look at cameras.size() instead of camera_config_count.
        if (!_create_cameras()) {
            _unload_cameras();
            return false;
        }
        if (!_check_cameras_connected()) {
            _unload_cameras();
            return false;
        }

        _start_cameras();

        //
        // start our run thread (which will drive the capturers and merge the pointclouds)
        //
        stopped = false;
        control_thread = new std::thread(&RS2BaseCapture::_control_thread_main, this);
        _cwipc_setThreadName(control_thread, L"cwipc_realsense2::RS2BaseCapture::control_thread");

        return true;
    }

    std::string config_get() override final {
        bool match_only = false;
        // We get parameters like exposure here.
        // But framerate and width/height are gotten in the camera code.
        _refresh_camera_hardware_parameters();
        for(auto cam : cameras) {
            bool ok = cam->getHardwareParameters(configuration.hardware, match_only);
            if (!ok) {
                if (!match_only) {
                    _log_warning("Could not get hardware parameters from first camera.");
                } else {
                    _log_warning("Not all cameras have the same hardware parameters.");
                }
            }
            match_only = true;
        }
        return configuration.to_string();
    }

    /// Tell the capturer that each point cloud should also include RGB and/or D images and/or RGB/D capture timestamps.
    virtual void request_auxiliary_data(bool rgb, bool depth, bool timestamps, bool skeleton) override final {
        configuration.auxData.want_auxdata_rgb = rgb;
        configuration.auxData.want_auxdata_depth = depth;
        configuration.auxData.want_auxdata_timestamps = timestamps;
    }

    //
    // This section has the public capturer-independent API used during normal runtime.
    //

    /// Returns true when a new point cloud is available.
    bool pointcloud_available(bool wait) override final {
        if (!is_valid()) {
            // xxxjack should we log a warning here?
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

    /// Returns the new point cloud. The caller is now the owner of this point cloud.
    cwipc* get_pointcloud() override final {
        if (!is_valid()) {
            _log_warning("get_pointcloud: returning NULL, no cameras");
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

    /// Returns a reasonable point size for the current capturer.
    float get_pointSize() override final  {
        if (!is_valid()) {
            // xxxjack should we log a warning here?
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

    /// Return 3D point for a given camera, given RGB image 2D coordinates.
    bool map2d3d(int tile, int x_2d, int y_2d, int d_2d, float* out3d) override final {
        for(auto cam : cameras) {
            if (tile == (1 << cam->camera_index)) {
                return cam->map2d3d(x_2d, y_2d, d_2d, out3d);
            }
        }
        return false;
    }

    /// Return 2D point in depth image coordinates given 2D point in color image coordinates.
    bool mapcolordepth(int tile, int u, int v, int* out2d) override final {
        for(auto cam : cameras) {
            if (tile == (1 << cam->camera_index)) {
                return cam->mapcolordepth(u, v, out2d);
            }
        }
        return false;
    }
    virtual bool eof() override final {
        return _eof;
    };
    /// Seek to given timestamp (only implemented for playback capturers).
    virtual bool seek(uint64_t timestamp) override = 0;
   
protected:
    /// Methods that are different for live vs playback capturers..
    /// Create the per-camera capturers.
    virtual bool _create_cameras() = 0;
    /// Setup camera synchronization (if needed).
    virtual bool _setup_inter_camera_sync() = 0;
    /// Setup camera hardware parameters (white balance, etc).
    virtual bool _init_hardware_for_all_cameras() = 0;
    /// Check that all cameras are connected.
    virtual bool _check_cameras_connected() = 0;
    
protected:
    /// Methods that are not different for live vs playback capturers..
    /// Get hardware parameters into our configuration structure.
    virtual void _refresh_camera_hardware_parameters() final {
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

    /// Unload all cameras and release all resources.
    void _unload_cameras()  {

        _stop_cameras();

        // Delete all cameras
        for (auto cam : cameras) {
            delete cam;
        }
        cameras.clear();
        _log_debug("deleted all cameras");
    }

    void _stop_cameras() {
        stopped = true;
        mergedPC_is_fresh = true;
        mergedPC_want_new = false;
        mergedPC_is_fresh_cv.notify_all();
        mergedPC_want_new = true;
        mergedPC_want_new_cv.notify_all();

        if (control_thread && control_thread->joinable()) {
            control_thread->join();
        }

        delete control_thread;
        control_thread = nullptr;

        // Stop all cameras
        for (auto cam : cameras) {
            cam->pre_stop_camera();
        }
        for (auto cam : cameras) {
            cam->stop_camera();
        }
        _log_debug("stopped all cameras");

    }
    
    Type_our_camera_config* get_camera_config(std::string serial) {
        for (int i = 0; i < configuration.all_camera_configs.size(); i++) {
            if (configuration.all_camera_configs[i].serial == serial) {
                return &configuration.all_camera_configs[i];
            }
        }

        _log_warning("Unknown camera " + serial);
        return nullptr;
    }

    virtual bool _capture_all_cameras(uint64_t& timestamp) final {
        uint64_t first_timestamp = 0;
        for(auto cam : cameras) {
            uint64_t this_cam_timestamp = cam->wait_for_captured_frameset(first_timestamp);
            if (first_timestamp == 0) {
                first_timestamp = this_cam_timestamp;
            }
        }

        // And get the best timestamp
        if (configuration.new_timestamps) {
            timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        } else if (timestamp == 0) {
            _log_warning("no timestamp obtained from cameras, using system time");
            timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        }
        return true;
    }

    void _control_thread_main()  {
        if (configuration.debug) _log_debug("processing thread started");
        _initial_camera_synchronization();
        while(!stopped) {
            {
                std::unique_lock<std::mutex> mylock(mergedPC_mutex);
                mergedPC_want_new_cv.wait(mylock, [this]{
                    return mergedPC_want_new;
                });
            }
            //check EOF:
            for (auto cam : cameras) {
                if (cam->end_of_stream_reached) {
                    _eof = true;
                    stopped = true;
                    break;
                }
            }

            if (stopped) {
                break;
            }

            assert (cameras.size() > 0);

            // Step one: grab frames from all cameras. This should happen as close together in time as possible,
            // because that gives use he biggest chance we have the same frame (or at most off-by-one) for each
            // camera.
            uint64_t timestamp = 0;
            bool all_captures_ok = _capture_all_cameras(timestamp);

            if (!all_captures_ok) {
                std::this_thread::yield();
                continue;
            }

            if (configuration.debug) _log_debug("creating pc with ts=" + std::to_string(timestamp));
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
                if (configuration.debug) _log_debug("cwipc_realsense2: merged pointcloud has " + std::to_string(mergedPC->access_pcl_pointcloud()->size()) + " points");
            } else {
                if (configuration.debug) _log_debug("cwipc_realsense2: Warning: capturer got an empty pointcloud");
            }
            // Signal that a new mergedPC is available. (Note that we acquired the mutex earlier)
            mergedPC_is_fresh = true;
            mergedPC_want_new = false;
            mergedPC_is_fresh_cv.notify_all();
        }

        if (configuration.debug) _log_debug_thread("cwipc_realsense2: processing thread exiting");
    }

    /// Anything that needs to be done to get the camera streams synchronized after opening.
    /// (Realsense Playback seeks all streams to the same timecode, the earliest one present
    /// in each stream)
    virtual void _initial_camera_synchronization() {
    }


    virtual void _start_cameras() final {
        bool start_error = false;
        for (auto cam: cameras) {
            if (!cam->pre_start_all_cameras()) {
                start_error = true;
            }
        }
        try {
            for (auto cam: cameras) {
                if (!cam->start_camera()) {
                    start_error = true;
                }
            }
        } catch(const rs2::error& e) {
            _log_error("Exception while starting camera: " + e.get_failed_function() + ": " + e.what());
            throw;
        }
        if (start_error) {
            _log_error("Not all cameras could be started");
            _unload_cameras();
            return;
        }
        //
        // start the per-camera capture threads
        //
        for (auto cam: cameras) {
            cam->start_camera_streaming();
        }
        
        for (auto cam: cameras) {
            cam->post_start_all_cameras();
        }
    }

    /// Load configuration from file or string.
    virtual bool _apply_config(const char* configFilename) {
        // Clear out old configuration
        RS2CaptureConfig newConfiguration;
        newConfiguration.auxData = configuration.auxData; // preserve auxdata requests
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
            return _apply_auto_config();
        }

        if (configFilename[0] == '{') {
            // Special case 2: a string starting with { is considered a JSON literal
            return configuration.from_string(configFilename, type);
        }

        // Otherwise we check the extension. It can be .json.
        const char *extension = strrchr(configFilename, '.');
        if (extension != nullptr && strcmp(extension, ".json") == 0) {
            return configuration.from_file(configFilename, type);
        }

        return false;
    }
    /// Load default configuration based on hardware cameras connected.
    virtual bool _apply_auto_config() = 0;


    void _request_new_pointcloud() {
        std::unique_lock<std::mutex> mylock(mergedPC_mutex);

        if (!mergedPC_want_new && !mergedPC_is_fresh) {
            mergedPC_want_new = true;
            mergedPC_want_new_cv.notify_all();
        }
    }

    void merge_camera_pointclouds() {
        cwipc_pcl_pointcloud aligned_cld(mergedPC->access_pcl_pointcloud());

        // Pre-allocate space in the merged pointcloud
        size_t nPoints = 0;

        for (auto cam : cameras) {
            cwipc_pcl_pointcloud cam_cld = cam->access_current_pcl_pointcloud();

            if (cam_cld == 0) {
                _log_warning("merge_camera_pointclouds: warning: camera pointcloud is null for one camera" );
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
public:
    /// Current configuration. Has to be public because cwipc_realsense2 needs access to all sorts of internals
    /// of it, but it would be better if this access was readonly...
    RS2CaptureConfig configuration;
    
protected:
    rs2::context capturer_context;

    std::vector<Type_our_camera*> cameras;    ///< The per-camera capturers
    bool stopped = false;
    bool _eof = false;

    cwipc* mergedPC = nullptr;          ///< Merged pointcloud
    std::mutex mergedPC_mutex;          ///< Lock for all mergedPC-related dta structures
    
    bool mergedPC_is_fresh = false;     ///< True if mergedPC contains a freshly-created pointcloud
    std::condition_variable mergedPC_is_fresh_cv;   ///< Condition variable for signalling freshly-created pointcloud
    
    bool mergedPC_want_new = false;     ///< Set to true to request a new pointcloud
    std::condition_variable mergedPC_want_new_cv;   ///< Condition variable for signalling we want a new pointcloud

    std::thread *control_thread = nullptr;
    
};

#endif