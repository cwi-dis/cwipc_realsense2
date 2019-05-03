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

/** \brief Capture pointclouds from realsense2 cameras.
 * \param errorMessage An optional pointer to a string where any error message will be stored.
 * \return A cwipc_source object.

 * This function returns a cwipc_source that captures pointclouds from realsense
 * cameras. If no camera is connected it will return "watermelon" pointclouds
 * similar to the `cwipc_synthetic()` source.
 */

_CWIPC_REALSENSE2_EXPORT cwipc_tiledsource* cwipc_realsense2_ex(const char *configFilename, char **errorMessage);

/** \brief Capture pointclouds from realsense2 cameras.
* \param configFilename An option string with the filename of the camera configuration file.
* \param errorMessage An optional pointer to a string where any error message will be stored.
* \return A cwipc_source object.
*
* This function returns a cwipc_source that captures pointclouds from realsense
* cameras. If no camera is connected it will return "watermelon" pointclouds
* similar to the `cwipc_synthetic()` source.
*/

_CWIPC_REALSENSE2_EXPORT cwipc_tiledsource* cwipc_realsense2(char **errorMessage);

#ifdef __cplusplus
};
#endif

#endif /* cwipc_realsense_api_h */
