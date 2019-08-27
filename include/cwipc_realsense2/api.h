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
/** \brief Converter to create pointclouds from streams of RGB and D images
 *
 * Note that the image data fed into the converter with feed() must be kept alive
 * until the resulting pointcloud has been retrieved with get_source()->get().
 */
class cwipc_offline {
public:
    virtual ~cwipc_offline() {};

    virtual void free() = 0;
	/** \brief Return the pointcloud source for this converter.
	 */
    virtual cwipc_tiledsource* get_source() = 0;

	/** \brief Feed an image into the converter.
	 * \param camNum Index of camera for which this data is meant.
	 * \param sensorNum Index of sensor within camera camNum for which this data is meant.
	 * \param buffer Pointer to the image data
	 * \param size Size of buffer (in bytes).
	 * \returns False in case of detectable errors (such as incorrect image size)
	 */
    virtual bool feed(int camNum, void *colorBuffer, size_t colorSize,  void *depthBuffer, size_t depthSize) = 0;
};
#else

/** \brief Abstract interface to a single pointcloud, C-compatible placeholder.
 */
typedef struct _cwipc_offline {
	int _dummy;
} cwipc_offline;

#endif

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Capture pointclouds from realsense2 cameras.
 * \param configFilename An option string with the filename of the camera configuration file.
 * \param errorMessage An optional pointer to a string where any error message will be stored.
 * \param apiVersion Pass in CWIPC_API_VERSION to ensure DLL compatibility.
 * \return A cwipc_source object.

 * This function returns a cwipc_source that captures pointclouds from realsense
 * cameras. If no camera is connected it will return "watermelon" pointclouds
 * similar to the `cwipc_synthetic()` source.
 */

_CWIPC_REALSENSE2_EXPORT cwipc_tiledsource* cwipc_realsense2(const char *configFilename, char **errorMessage, uint64_t apiVersion);

/** \brief Capture pointclouds from offline realsense2 images.
 * \param configFilename An option string with the filename of the camera configuration file.
 * \param errorMessage An optional pointer to a string where any error message will be stored.
 * \param apiVersion Pass in CWIPC_API_VERSION to ensure DLL compatibility.
 * \return A cwipc_source object.

 * This function returns a cwipc_source that create pointclouds from color and
 * depth images captured earlier (or elsewhere) from realsense
 * cameras.
 */

_CWIPC_REALSENSE2_EXPORT cwipc_offline* cwipc_rs2offline(const char *configFilename, char **errorMessage, uint64_t apiVersion);

#ifdef __cplusplus
};
#endif

#endif /* cwipc_realsense_api_h */
