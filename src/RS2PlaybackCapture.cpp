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
            cwipc_rs2_log_warning("Camera " + cd.serial + " is type realsense, override to realsense_playback");
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