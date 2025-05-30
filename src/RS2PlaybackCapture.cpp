//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//
#include <cstdlib>

// Define to get (a little) debug prints
#undef CWIPC_DEBUG
#undef CWIPC_DEBUG_THREAD

// This is the dll source, so define external symbols as dllexport on windows.

#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif

#include "cwipc_realsense2/private/RS2PlaybackCapture.hpp"

RS2PlaybackCapture::RS2PlaybackCapture()
:   RS2Capture()
{
    type = "realsense_playback";
}

RS2PlaybackCapture::~RS2PlaybackCapture() {
}

bool RS2PlaybackCapture::_apply_config(const char* configFilename) {
    bool ok = RS2Capture::_apply_config(configFilename);
    if (!ok) {
        return ok;
    }
    if ( configFilename != NULL && configFilename[0] != '{' && (
        strchr(configFilename, '/') != NULL
#ifdef WIN32
        || strchr(configFilename, '\\') != NULL
#endif
        )) {
        // It appears the configFilename is a pathname. Get the directory component.
        std::string dirName(configFilename);
        int slashPos = dirName.find_last_of("/\\");
        base_directory = dirName.substr(0, slashPos+1);
    }
    return true;
}


bool RS2PlaybackCapture::_create_cameras() {
    for (RS2CameraConfig& cd : configuration.all_camera_configs) {
        // Ensure we have a serial
        if (cd.serial == "") {
            cd.serial = cd.playback_filename;
        }
        if (cd.playback_filename == "" && cd.serial == "") {
            cwipc_rs2_log_warning("Camera " + cd.serial + " has no playback_filename nor serial");
            return false;
        }
        if (cd.type == "realsense"){
            // cwipc_rs2_log_warning("Camera " + cd.serial + " is type realsense, override to realsense_playback");
            cd.type = "realsense_playback";
        }
        if (cd.type != "realsense_playback") {
            cwipc_rs2_log_warning("Camera " + cd.serial + " is type " + cd.type + " in stead of realsense_playback");
            return false;
        }

        int camera_index = (int)cameras.size();

        if (cd.disabled) {
            // xxxnacho do we need to close the device, like the kinect case?
        }
        else {
            std::string recording_filename = cd.playback_filename;
            if (recording_filename == "") recording_filename = cd.serial + ".bag";
            if (base_directory != "") {
                recording_filename = base_directory + recording_filename;
            }
            auto cam = new RS2PlaybackCamera(capturer_context, configuration, camera_index, recording_filename);
            cameras.push_back(cam);
            cd.connected = true;
        }

    }
    return true;
}


void RS2PlaybackCapture::_initial_camera_synchronization() {
    uint64_t newest_first_timestamp = 0;
    // Find the newest timestamp (the camera that started last)
    // std::cerr << "xxxjack search first timestamp" << std::endl;
    for(auto cam : cameras) {
        uint64_t this_cam_timestamp = cam->wait_for_captured_frameset(0);
        if (earliest_recording_timestamp_seen == 0) {
            earliest_recording_timestamp_seen = this_cam_timestamp;
        }
        if (this_cam_timestamp > newest_first_timestamp) {
            newest_first_timestamp = this_cam_timestamp;
        }
    }
    // std::cerr << "xxxjack first timestamp=" << newest_first_timestamp << std::endl;
    // Seek all cameras to that timestamp
    for(auto cam : cameras) {
        uint64_t this_cam_timestamp = cam->wait_for_captured_frameset(newest_first_timestamp);
        // std::cerr << "xxxjack camera after seek ts=" << this_cam_timestamp << std::endl;
    }
}

bool RS2PlaybackCapture::seek(uint64_t timestamp) {
    if (earliest_recording_timestamp_seen == 0) {
        std::cerr << "RS2PlaybackCapture: no timestamps seen yet, seek() may fail" << std::endl;
    }
    uint64_t delta = timestamp - earliest_recording_timestamp_seen;
    // std::cerr << "xxxjack RS2PlaybackCapture: pausing cameras" << std::endl;
    for (auto cam : cameras) {
        RS2PlaybackCamera* pbcam = dynamic_cast<RS2PlaybackCamera*>(cam);
        if (pbcam == nullptr) {
            std::cerr << "RS2PlaybackCapture: Camera " << cam->serial << " is not a RS2PlaybackCamera" << std::endl;
            return false;
        }
        pbcam->pause();
    }
    // std::cerr << "xxxjack RS2PlaybackCapture: seek to timestamp " << timestamp << " delta " << delta << std::endl;
    for (auto cam : cameras) { //SUBORDINATE or STANDALONE
        if (cam->seek(delta) != true) {
            std::cerr << "RS2PlaybackCapture: Camera " << cam->serial << " failed to seek to delta-timestamp " << delta << std::endl;
            return false;
        }
    }
    // std::cerr << "xxxjack RS2PlaybackCapture: resuming cameras" << std::endl;
    for (auto cam : cameras) {
        RS2PlaybackCamera* pbcam = dynamic_cast<RS2PlaybackCamera*>(cam);
        if (pbcam == nullptr) {
            std::cerr << "RS2PlaybackCapture: Camera " << cam->serial << " is not a RS2PlaybackCamera" << std::endl;
            return false;
        }
        pbcam->resume();
    }
    // std::cerr << "xxxjack RS2PlaybackCapture: seek done." << std::endl;

#if 0
    _initial_camera_synchronization();
#endif

    return true;
} 

