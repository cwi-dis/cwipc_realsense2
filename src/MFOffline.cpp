//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//

// Define to get (a little) debug prints
#undef CWIPC_DEBUG
#undef CWIPC_DEBUG_THREAD

// This is the dll source, so define external symbols as dllexport on windows.

#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif

#include "cwipc_realsense2/defs.h"
#include "cwipc_realsense2/utils.h"
#include "cwipc_realsense2/MFOffline.hpp"
#include "cwipc_realsense2/MFOfflineCamera.hpp"

MFOffline::MFOffline(const char *configFilename)
:	MFCapture(configFilename)
{

}

MFOffline::~MFOffline() {

}

void MFOffline::_create_cameras(rs2::device_list devs) {
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
		auto cam = new MFOfflineCamera(ctx, configuration, camera_index, cd);
		cameras.push_back(cam);
	}
}
