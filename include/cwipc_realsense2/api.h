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
class cwipc_source {
public:
    virtual ~cwipc_source() {};
    virtual void free() = 0;
    virtual cwipc* get() = 0;
};
#else
typedef struct _cwipc_source {
    int _dummy;
} cwipc_source;
#endif

#ifdef __cplusplus
extern "C" {
#endif

_CWIPC_REALSENSE2_EXPORT cwipc_source* cwipc_realsense2(char **errorMessage);

_CWIPC_REALSENSE2_EXPORT cwipc* cwipc_source_get(cwipc_source *);
_CWIPC_REALSENSE2_EXPORT void cwipc_source_free(cwipc_source *);

#ifdef __cplusplus
};
#endif

#endif /* cwipc_realsense_api_h */
