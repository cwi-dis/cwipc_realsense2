#ifndef cwipc_realsense_MFOffline_hpp
#define cwipc_realsense_MFOffline_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

#include <librealsense2/rs.hpp>

#include "defs.h"
#include "cwipc_realsense2/MFCapture.hpp"

#ifndef CWIPC_DLL_ENTRY
#if defined(WIN32) || defined(_WIN32)
#define CWIPC_DLL_ENTRY __declspec(dllimport)
#else
#define CWIPC_DLL_ENTRY 
#endif
#endif

class CWIPC_DLL_ENTRY MFOffline : public MFCapture {

public:
	// methods
	MFOffline(const char *configFilename=NULL);
	~MFOffline();
private:
	virtual void _create_cameras(rs2::device_list devs);
};
#endif // cwipc_realsense_MFOffline_hpp
