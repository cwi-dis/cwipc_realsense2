#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif
#include <inttypes.h>

#include "cwipc_util/api_pcl.h"
#include "cwipc_util/api.h"
#include "cwipc_realsense2/api.h"

#include "cwipc_realsense2/private/RS2Config.hpp"
#include "cwipc_realsense2/private/RS2Capture.hpp"
#include "cwipc_realsense2/private/RS2Camera.hpp"
#include "cwipc_realsense2/private/RS2PlaybackCapture.hpp"
#include "cwipc_realsense2/private/RS2PlaybackCamera.hpp"

// Global variables (constants, really)

_CWIPC_REALSENSE2_EXPORT int CWIPC_RS2_FORMAT_Z16 = RS2_FORMAT_Z16;
_CWIPC_REALSENSE2_EXPORT int CWIPC_RS2_FORMAT_RGB8 = RS2_FORMAT_RGB8;

static bool rs2_versioncheck(char **errorMessage) {
    int version = rs2_get_api_version(nullptr);
    if ((version/100) == (RS2_API_VERSION/100)) {
        return true;
    }

    if (errorMessage) {
        static char errorBuf[80];
        snprintf(errorBuf, sizeof(errorBuf), "Built against librealsense %d but %d is installed.", RS2_API_VERSION, version);
        *errorMessage = errorBuf;
    }

    return false;
}

static cwipc_vector* add_vectors(cwipc_vector a, cwipc_vector b, cwipc_vector *result) {
    if (result) {
        result->x = a.x + b.x;
        result->y = a.y + b.y;
        result->z = a.z + b.z;
    }

    return result;
}

static cwipc_vector* diff_vectors(cwipc_vector a, cwipc_vector b, cwipc_vector *result) {
    if (result) {
        result->x = a.x - b.x;
        result->y = a.y - b.y;
        result->z = a.z - b.z;
    }

    return result;
}

static double len_vector(cwipc_vector v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

static cwipc_vector* mult_vector(double factor, cwipc_vector *v) {
    if (v) {
        v->x *= factor;
        v->y *= factor;
        v->z *= factor;
    }

    return v;
}

static cwipc_vector* norm_vector(cwipc_vector *v) {
    double len = len_vector(*v);

    if (len > 0) {
        mult_vector(1.0/len, v);
    }

    return v;
}

static double dot_vectors(cwipc_vector a, cwipc_vector b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static cwipc_vector* cross_vectors(cwipc_vector a, cwipc_vector b, cwipc_vector *result) {
    if (result) {
        result->x = a.y*b.z - a.z*b.y;
        result->y = a.z*b.x - a.x*b.z;
        result->z = a.x*b.y - a.y*b.x;
    }

    return result;
}

class cwipc_source_realsense2_impl : public cwipc_tiledsource {
    friend class cwipc_source_realsense2_playback_impl;
protected:
    RS2Capture *m_grabber;
    cwipc_source_realsense2_impl(RS2Capture *obj) : m_grabber(obj) {}

public:
    cwipc_source_realsense2_impl(const char *configFilename=NULL) : m_grabber(RS2Capture::factory()) {
        m_grabber->config_reload(configFilename);
    }

    ~cwipc_source_realsense2_impl() {
        delete m_grabber;
        m_grabber = NULL;
    }

    bool is_valid() {
        return m_grabber->camera_count > 0;
    }

    void free() override {
        delete m_grabber;
        m_grabber = NULL;
    }

    virtual size_t get_config(char* buffer, size_t size) override {
        auto config = m_grabber->config_get();

        if (buffer == nullptr) {
            return config.length();
        }

        if (size < config.length()) {
            return 0;
        }

        memcpy(buffer, config.c_str(), config.length());
        return config.length();
    }

    virtual bool reload_config(const char* configFile) override {
        return m_grabber->config_reload(configFile);
    }

    bool eof() override {
        return false;
    }

    bool available(bool wait) override {
        if (m_grabber == NULL) {
            return false;
        }

        return m_grabber->pointcloud_available(wait);
    }

    cwipc* get() override {
        if (m_grabber == NULL) {
            return NULL;
        }

        cwipc* rv = m_grabber->get_pointcloud();
        return rv;
    }

    int maxtile() override {
        if (m_grabber == NULL) {
            return 0;
        }

        int nCamera = (int)m_grabber->configuration.all_camera_configs.size();

        if (nCamera <= 1) {
            // Using a single camera or no camera.
            return nCamera;
        }

        return nCamera+1;
    }

    bool get_tileinfo(int tilenum, struct cwipc_tileinfo *tileinfo) override {
        if (m_grabber == NULL) {
            return false;
        }

        int nCamera = (int)m_grabber->configuration.all_camera_configs.size();

        if (nCamera == 0) { // No camera
            return false;
        }

        if (tilenum < 0 || tilenum >= nCamera+1) {
            return false;
        }

        if (tilenum == 0) {
            // Special case: the whole pointcloud
            if (tileinfo) {
                tileinfo->normal = { 0, 0, 0 };
                tileinfo->cameraName = NULL;
                tileinfo->ncamera = nCamera;
                tileinfo->cameraMask = 0; // All cameras contributes to this
            }
            return true;
        }
        RS2CameraConfig &cameraConfig = m_grabber->configuration.all_camera_configs[tilenum-1];
        if (tileinfo) {
            tileinfo->normal = cameraConfig.cameraposition; // Use the camera position as the normal
            tileinfo->cameraName = (char *)cameraConfig.serial.c_str();
            tileinfo->ncamera = 1; // Only one camera contributes to this
            tileinfo->cameraMask = (uint8_t)1 << (tilenum-1); // Only this camera contributes
        }
        return true;
    }

    void request_auxiliary_data(const std::string& name) override {
        cwipc_tiledsource::request_auxiliary_data(name);
        m_grabber->request_auxdata(auxiliary_data_requested("rgb"), auxiliary_data_requested("depth"), auxiliary_data_requested("timestamps"));
    }

    bool auxiliary_operation(const std::string op, const void* inbuf, size_t insize, void* outbuf, size_t outsize) override {
        if (op == "map2d3d") {
            if (inbuf == nullptr || insize != 4*sizeof(float)) return false;
            if (outbuf == nullptr || outsize != 3*sizeof(float)) return false;
            float *infloat = (float *)inbuf;
            float *outfloat = (float *)outbuf;
            int tilenum = (int)infloat[0];
            int x_2d = (int)infloat[1];
            int y_2d = (int)infloat[2];
            float d_2d = infloat[3];

            return m_grabber->map2d3d(tilenum, x_2d, y_2d, d_2d, outfloat);
        } else if (op == "mapcolordepth") {
            if (inbuf == nullptr || insize != 3*sizeof(int)) return false;
            if (outbuf == nullptr || outsize != 2*sizeof(int)) return false;
            int *inint = (int *)inbuf;
            int *outint = (int *)outbuf;

            return m_grabber->mapcolordepth(inint[0], inint[1], inint[2], outint);

        } else {
            return false;
        }
    }
};

class cwipc_source_realsense2_playback_impl : public cwipc_tiledsource {
    // xxxjack this is stupid: this is a verbatim copy of cwipc_source_realsense2_impl except for the type of m_grabber.
protected:
    RS2PlaybackCapture *m_grabber;
    cwipc_source_realsense2_playback_impl(RS2PlaybackCapture *obj) : m_grabber(obj) {}

public:
    cwipc_source_realsense2_playback_impl(const char *configFilename=NULL)
    :   m_grabber(RS2PlaybackCapture::factory()) 
    {
        m_grabber->config_reload(configFilename);
    }

    ~cwipc_source_realsense2_playback_impl() {
        delete m_grabber;
        m_grabber = NULL;
    }

    bool is_valid() {
        return m_grabber->camera_count > 0;
    }

    void free() override {
        delete m_grabber;
        m_grabber = NULL;
    }

    virtual size_t get_config(char* buffer, size_t size) override {
        auto config = m_grabber->config_get();

        if (buffer == nullptr) {
            return config.length();
        }

        if (size < config.length()) {
            return 0;
        }

        memcpy(buffer, config.c_str(), config.length());
        return config.length();
    }

    virtual bool reload_config(const char* configFile) override {
        return m_grabber->config_reload(configFile);
    }

    bool eof() override {
        return false;
    }

    bool available(bool wait) override {
        if (m_grabber == NULL) {
            return false;
        }

        return m_grabber->pointcloud_available(wait);
    }

    cwipc* get() override {
        if (m_grabber == NULL) {
            return NULL;
        }

        cwipc* rv = m_grabber->get_pointcloud();
        return rv;
    }

    int maxtile() override {
        if (m_grabber == NULL) {
            return 0;
        }

        int nCamera = (int)m_grabber->configuration.all_camera_configs.size();

        if (nCamera <= 1) {
            // Using a single camera or no camera.
            return nCamera;
        }

        return nCamera+1;
    }

    bool get_tileinfo(int tilenum, struct cwipc_tileinfo *tileinfo) override {
        if (m_grabber == NULL) {
            return false;
        }

        int nCamera = (int)m_grabber->configuration.all_camera_configs.size();

        if (nCamera == 0) { // No camera
            return false;
        }

        if (tilenum < 0 || tilenum >= (1<<nCamera)) {
            return false;
        }

        // nCamera > 0
        cwipc_vector camcenter = { 0, 0, 0 };

        // calculate the center of all cameras
        for (auto camdat : m_grabber->configuration.all_camera_configs) {
            add_vectors(camcenter, camdat.cameraposition, &camcenter);
        }
        mult_vector(1.0 / nCamera, &camcenter);

        // calculate normalized direction vectors from the center towards each camera
        std::vector<cwipc_vector> camera_directions;
        for (auto camdat : m_grabber->configuration.all_camera_configs) {
            cwipc_vector normal;
            diff_vectors(camdat.cameraposition, camcenter, &normal);
            norm_vector(&normal);
            camera_directions.push_back(normal);
        }

        // add all cameradirections that contributed
        int ncontribcam = 0;
        int lastcontribcamid = 0;
        cwipc_vector tile_direction = { 0, 0, 0 };
        for (int i = 0; i < m_grabber->configuration.all_camera_configs.size(); i++) {
            uint8_t camera_label = (uint8_t)1 << i;

            if (tilenum == 0 || (tilenum & camera_label)) {
                add_vectors(tile_direction, camera_directions[i], &tile_direction);
                ncontribcam++;
                lastcontribcamid = i;
            }
        }
        norm_vector(&tile_direction);

        if (tileinfo) {
            tileinfo->normal = tile_direction;
            tileinfo->cameraName = NULL;
            tileinfo->ncamera = ncontribcam;
            tileinfo->cameraMask = tilenum;

            if (ncontribcam == 1) {
                // A single camera contributed to this
                tileinfo->cameraName = (char *)m_grabber->configuration.all_camera_configs[lastcontribcamid].serial.c_str();
            }
        }

        return true;
    }

    void request_auxiliary_data(const std::string& name) override {
        cwipc_tiledsource::request_auxiliary_data(name);
        m_grabber->request_auxdata(auxiliary_data_requested("rgb"), auxiliary_data_requested("depth"), auxiliary_data_requested("timestamps"));
   }

    bool auxiliary_operation(const std::string op, const void* inbuf, size_t insize, void* outbuf, size_t outsize) override {
        if (op == "map2d3d") {
            if (inbuf == nullptr || insize != 4*sizeof(float)) return false;
            if (outbuf == nullptr || outsize != 3*sizeof(float)) return false;
            float *infloat = (float *)inbuf;
            float *outfloat = (float *)outbuf;
            int tilenum = (int)infloat[0];
            int x_2d = (int)infloat[1];
            int y_2d = (int)infloat[2];
            float d_2d = infloat[3];

            return m_grabber->map2d3d(tilenum, x_2d, y_2d, d_2d, outfloat);
        } else if (op == "mapcolordepth") {
            if (inbuf == nullptr || insize != 3*sizeof(int)) return false;
            if (outbuf == nullptr || outsize != 2*sizeof(int)) return false;
            int *inint = (int *)inbuf;
            int *outint = (int *)outbuf;

            return m_grabber->mapcolordepth(inint[0], inint[1], inint[2], outint);

        } else {
            return false;
        }
    }

    bool seek(uint64_t timestamp) override {
        if (m_grabber == NULL) {
            return false;
        }

        bool rv = m_grabber->seek(timestamp);
        return rv;
    }
};

//
// C-compatible entry points
//

cwipc_tiledsource* cwipc_realsense2(const char *configFilename, char **errorMessage, uint64_t apiVersion) {
    if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
        if (errorMessage) {
            char* msgbuf = (char*)malloc(1024);
            snprintf(msgbuf, 1024, "cwipc_realsense2: incorrect apiVersion 0x%08" PRIx64 " expected 0x%08" PRIx64 "..0x%08" PRIx64 "", apiVersion, CWIPC_API_VERSION_OLD, CWIPC_API_VERSION);
            *errorMessage = msgbuf;
        }

        return NULL;
    }

    if (!rs2_versioncheck(errorMessage)) {
        return NULL;
    }

    cwipc_rs2_warning_store = errorMessage;
    cwipc_source_realsense2_impl *rv = new cwipc_source_realsense2_impl(configFilename);
    cwipc_rs2_warning_store = NULL;

    // If the grabber found cameras everything is fine
    if (rv && rv->is_valid()) {
        return rv;
    }

    delete rv;

    if (errorMessage && *errorMessage == NULL) {
        *errorMessage = (char *)"cwipc_realsense2: no realsense cameras found";
    }

    return NULL;
}


cwipc_tiledsource* cwipc_realsense2_playback(const char *configFilename, char **errorMessage, uint64_t apiVersion) {
    if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
        if (errorMessage) {
            char* msgbuf = (char*)malloc(1024);
            snprintf(msgbuf, 1024, "cwipc_realsense2_playback: incorrect apiVersion 0x%08" PRIx64 " expected 0x%08" PRIx64 "..0x%08" PRIx64 "", apiVersion, CWIPC_API_VERSION_OLD, CWIPC_API_VERSION);
            *errorMessage = msgbuf;
        }

        return NULL;
    }

    if (!rs2_versioncheck(errorMessage)) {
        return NULL;
    }

    cwipc_rs2_warning_store = errorMessage;
    cwipc_source_realsense2_playback_impl *rv = new cwipc_source_realsense2_playback_impl(configFilename);
    cwipc_rs2_warning_store = NULL;

    // If the grabber found cameras everything is fine
    if (rv && rv->is_valid()) {
        return rv;
    }

    delete rv;

    if (errorMessage && *errorMessage == NULL) {
        *errorMessage = (char *)"cwipc_realsense2_playback: unspecified error from constructor";
    }

    return NULL;
}


//
// These static variables only exist to ensure the initializer is called, which registers our camera type.
//
int _cwipc_dummy_realsense_initializer = _cwipc_register_capturer("realsense", RS2Capture::count_devices, cwipc_realsense2);
int _cwipc_dummy_realsense_playback_initializer = _cwipc_register_capturer("realsense_playback", RS2PlaybackCapture::count_devices, cwipc_realsense2_playback);
