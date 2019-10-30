#ifndef cwipc_realsense_api_h
#define cwipc_realsense_api_h

#include "cwipc_util/api.h"
#include "offlinedefs.h"

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
	 * \param frameNum frame number for this data
	 * \param colorBuffer Pointer to the RGB image data
	 * \param colorSize Size of colorBuffer (in bytes).
	 * \param depthBuffer Pointer to the depth data
	 * \param depthSize Size of depthBuffer (in bytes).
	 * \returns False in case of detectable errors (such as incorrect image size)
	 */
    virtual bool feed(int camNum, int frameNum, void *colorBuffer, size_t colorSize,  void *depthBuffer, size_t depthSize) = 0;
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

extern _CWIPC_REALSENSE2_EXPORT int CWIPC_RS2_FORMAT_RGB8;	//!< Constant: 24-bit RGB pixels
extern _CWIPC_REALSENSE2_EXPORT int CWIPC_RS2_FORMAT_Z16;	//!< Constant: 16-bit Depth pixels

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
 * \param settings Structure containing settings for the offline capturer
 * \param configFilename An option string with the filename of the camera configuration file.
 * \param errorMessage An optional pointer to a string where any error message will be stored.
 * \param apiVersion Pass in CWIPC_API_VERSION to ensure DLL compatibility.
 * \return A cwipc_offline object.

 * This function returns a cwipc_source that create pointclouds from color and
 * depth images captured earlier (or elsewhere) from realsense
 * cameras.
 */

_CWIPC_REALSENSE2_EXPORT cwipc_offline* cwipc_rs2offline(MFOfflineSettings settings, const char *configFilename, char **errorMessage, uint64_t apiVersion);

/** \brief Feed image data into an offline pointcloud constructor.
 * \param obj The cwipc_offline object to feed data to.
 * \param camNum Camera number from which these images were taken
 * \param frameNum Frame number (index) of this data
 * \param colorBuffer Pointer to image data
 * \param colorSize Length of colorBuffer in bytes
 * \param depthBuffer Pointer to depth data
 * \param depthSize Length of depth buffer in bytes
 * \return Success indicator

 * This function feeds image and depth data for one virtual camera into the pointcloud constructor.
 * After all cameras have been fed data the get() method can be used to retrieve the resulting pointcloud.
 */

_CWIPC_REALSENSE2_EXPORT bool cwipc_offline_feed(cwipc_offline* obj, int camNum, int frameNum, void *colorBuffer, size_t colorSize, void *depthBuffer, size_t depthSize);

/** \brief Free the offline converter.
 */
_CWIPC_REALSENSE2_EXPORT void cwipc_offline_free(cwipc_offline* obj, int camNum, void *colorBuffer, size_t colorSize, void *depthBuffer, size_t depthSize);

/** \brief Return the pointcloud source for this converter.
 */
_CWIPC_REALSENSE2_EXPORT cwipc_tiledsource* cwipc_offline_get_source(cwipc_offline* obj);
#ifdef __cplusplus
};
#endif

#endif /* cwipc_realsense_api_h */
