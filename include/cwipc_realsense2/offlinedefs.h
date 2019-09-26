//
//  defs.h
//
//  Created by Fons Kuijk on 12-12-18.
//

#ifndef cwipc_realsense2_offlinedefs_h
#define cwipc_realsense2_offlinedefs_h

struct MFOfflineCameraSettings {
	int width;
	int height;
	int bpp;
	int fps;
	int format;
};

struct MFOfflineSettings {
	struct MFOfflineCameraSettings color;
	struct MFOfflineCameraSettings depth;
};

#endif /* cwipc_realsense2_offlinedefs_h */
