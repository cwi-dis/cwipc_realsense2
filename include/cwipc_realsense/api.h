#ifndef cwipc_realsense_api_h
#define cwipc_realsense_api_h

/* Ensure we have the right dllexport or dllimport on windows */
#ifndef CWIPC_DLL_ENTRY
#if defined(WIN32) || defined(_WIN32)
#define CWIPC_DLL_ENTRY __declspec(dllimport)
#else
#define CWIPC_DLL_ENTRY 
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

void CWIPC_DLL_ENTRY getPointCloud(uint64_t *timestamp, void **pointcloud);

#ifdef __cplusplus
};
#endif

#endif /* cwipc_realsense_api_h */