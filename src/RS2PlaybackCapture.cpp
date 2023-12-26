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

bool RS2PlaybackCapture::config_reload(const char* configFilename) {
    return RS2Capture::config_reload(configFilename);
}


bool RS2PlaybackCapture::_create_cameras() {
    assert(false);
    rs2::device_list devs = ctx().query_devices();
    const std::string platform_camera_name = "Platform Camera";

    for (auto dev : devs) {
        if (dev.get_info(RS2_CAMERA_INFO_NAME) == platform_camera_name) {
            continue;
        }

#ifdef CWIPC_DEBUG
        std::cout << "cwipc_realsense2: opening camera " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
#endif

        // Found a realsense camera. Create a default data entry for it.
        std::string serial(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        std::string camUsb(dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR));

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
            auto cam = new RS2Camera(ctx(), configuration, camera_index, *cd, camUsb);
            cameras.push_back(cam);
            cd->connected = true;
        }
    }

    return true;
}