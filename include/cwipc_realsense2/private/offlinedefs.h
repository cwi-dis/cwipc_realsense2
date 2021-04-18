//
//  defs.h
//
//  Created by Fons Kuijk on 12-12-18.
//

#ifndef cwipc_realsense2_offlinedefs_h
#define cwipc_realsense2_offlinedefs_h

struct RS2OfflineCameraSettings {
	int width;
	int height;
	int bpp;
	int fps;
	int format;
};

struct RS2OfflineSettings {
	struct RS2OfflineCameraSettings color;
	struct RS2OfflineCameraSettings depth;
};

#endif /* cwipc_realsense2_offlinedefs_h */
