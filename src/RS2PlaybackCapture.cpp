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

}

RS2PlaybackCapture::~RS2PlaybackCapture() {
}

bool RS2PlaybackCapture::config_reload(const char* configFilename) {
    RS2Capture::config_reload(configFilename);
}
