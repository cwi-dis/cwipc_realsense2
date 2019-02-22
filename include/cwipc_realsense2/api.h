#ifndef cwipc_realsense_api_h
#define cwipc_realsense_api_h

#include "cwipc_util/api.h"

/* Ensure we have the right dllexport or dllimport on windows */
#ifndef _CWIPC_REALSENSE2_EXPORT
#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllimport)
#else
#define _CWIPC_REALSENSE2_EXPORT 
#endif
#endif

#ifdef __cplusplus
#else
#endif

#ifdef __cplusplus
extern "C" {
#endif

_CWIPC_REALSENSE2_EXPORT cwipc_source* cwipc_realsense2(char **errorMessage);

#ifdef __cplusplus
};
#endif

#endif /* cwipc_realsense_api_h */
