//
//  utils.cpp
//
//  Created by Fons Kuijk on 12-12-18.
//
#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif
#include "RS2Config.hpp"

#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#define _MY_JSON_GET(jsonobj, name, config, attr) if (jsonobj.contains(#name)) jsonobj.at(#name).get_to(config.attr)
#define _MY_JSON_PUT(jsonobj, name, config, attr) jsonobj[#name] = config.attr

static std::string cwipc_rs2_most_recent_warning;
char **cwipc_rs2_warning_store;

void cwipc_rs2_log_warning(std::string warning) {
    std::cerr << "cwipc_realsense2: Warning: " << warning << std::endl;

    if (cwipc_rs2_warning_store) {
        cwipc_rs2_most_recent_warning = warning;
        *cwipc_rs2_warning_store = (char *)cwipc_rs2_most_recent_warning.c_str();
    }
}

void from_json(const json& json_data, RS2CaptureConfig& config) {
    // version and type should already have been checked.
    RS2CameraHardwareConfig& hardware(config.hardware);
    RS2CameraProcessingParameters& filtering(config.filtering);
    RS2CaptureProcessingConfig& processing(config.processing);
    RS2CaptureSyncConfig& sync(config.sync);

    json system_data = json_data.at("system");
    _MY_JSON_GET(system_data, record_to_directory, config, record_to_directory);
    _MY_JSON_GET(system_data, playback_realtime, config, playback_realtime);
    _MY_JSON_GET(system_data, new_timestamps, config, new_timestamps);
    _MY_JSON_GET(system_data, debug, config, debug);
    _MY_JSON_GET(system_data, prefer_color_timing, config, prefer_color_timing);

    json sync_data = json_data.at("sync");
    _MY_JSON_GET(sync_data, sync_master_serial, sync, sync_master_serial);
    _MY_JSON_GET(sync_data, sync_mode, sync, sync_mode);

    json hardware_data = json_data.at("hardware");
    _MY_JSON_GET(hardware_data, color_width, hardware, color_width);
    _MY_JSON_GET(hardware_data, color_height, hardware, color_height);
    _MY_JSON_GET(hardware_data, fps, hardware, fps);
    _MY_JSON_GET(hardware_data, depth_width, hardware, depth_width);
    _MY_JSON_GET(hardware_data, depth_height, hardware, depth_height);
    _MY_JSON_GET(hardware_data, fps, hardware, fps);
    _MY_JSON_GET(hardware_data, visual_preset, hardware, visual_preset);
    _MY_JSON_GET(hardware_data, color_exposure, hardware, color_exposure);
    _MY_JSON_GET(hardware_data, color_gain, hardware, color_gain);
    _MY_JSON_GET(hardware_data, depth_exposure, hardware, depth_exposure);
    _MY_JSON_GET(hardware_data, depth_gain, hardware, depth_gain);
    _MY_JSON_GET(hardware_data, whitebalance, hardware, whitebalance);
    _MY_JSON_GET(hardware_data, backlight_compensation, hardware, backlight_compensation);
    _MY_JSON_GET(hardware_data, laser_power, hardware, laser_power);

    json processing_data = json_data.at("processing");
    _MY_JSON_GET(processing_data, greenscreen_removal, processing, greenscreen_removal);
    _MY_JSON_GET(processing_data, height_min, processing, height_min);
    _MY_JSON_GET(processing_data, height_max, processing, height_max);
    _MY_JSON_GET(processing_data, radius_filter, processing, radius_filter);
    _MY_JSON_GET(processing_data, depth_x_erosion, processing, depth_x_erosion);
    _MY_JSON_GET(processing_data, depth_y_erosion, processing, depth_y_erosion);

    json filtering_data = json_data.at("filtering");
    _MY_JSON_GET(filtering_data, map_color_to_depth, filtering, map_color_to_depth);
    _MY_JSON_GET(filtering_data, do_decimation, filtering, do_decimation);
    _MY_JSON_GET(filtering_data, decimation_magnitude, filtering, decimation_magnitude);
    _MY_JSON_GET(filtering_data, do_threshold, filtering, do_threshold);
    _MY_JSON_GET(filtering_data, threshold_min_distance, filtering, threshold_min_distance);
    _MY_JSON_GET(filtering_data, threshold_max_distance, filtering, threshold_max_distance);
    _MY_JSON_GET(filtering_data, do_spatial, filtering, do_spatial);
    _MY_JSON_GET(filtering_data, spatial_magnitude, filtering, spatial_magnitude);
    _MY_JSON_GET(filtering_data, spatial_smooth_alpha, filtering, spatial_smooth_alpha);
    _MY_JSON_GET(filtering_data, spatial_smooth_delta, filtering, spatial_smooth_delta);
    _MY_JSON_GET(filtering_data, spatial_holes_fill, filtering, spatial_holes_fill);
    _MY_JSON_GET(filtering_data, do_temporal, filtering, do_temporal);
    _MY_JSON_GET(filtering_data, temporal_smooth_alpha, filtering, temporal_smooth_alpha);
    _MY_JSON_GET(filtering_data, temporal_smooth_delta, filtering, temporal_smooth_delta);
    _MY_JSON_GET(filtering_data, temporal_persistency, filtering, temporal_persistency);
    _MY_JSON_GET(filtering_data, do_hole_filling, filtering, do_hole_filling);
    _MY_JSON_GET(filtering_data, hole_filling_mode, filtering, hole_filling_mode);

    json cameras = json_data.at("camera");
    int camera_index = 0;
    config.all_camera_configs.clear();

    for(json::iterator it=cameras.begin(); it != cameras.end(); it++) {
        json camera = *it;
        RS2CameraConfig cd;
        pcl::shared_ptr<Eigen::Affine3d> default_trafo(new Eigen::Affine3d());

        default_trafo->setIdentity();
        cd.trafo = default_trafo;
        
        _MY_JSON_GET(camera, serial, cd, serial);
        _MY_JSON_GET(camera, disabled, cd, disabled);
        _MY_JSON_GET(camera, playback_filename, cd, playback_filename);
        _MY_JSON_GET(camera, playback_inpoint_micros, cd, playback_inpoint_micros);
        _MY_JSON_GET(camera, type, cd, type);

        if (camera.contains("trafo")) {
            for (int x=0; x<4; x++) {
                for (int y=0; y<4; y++) {
                    (*cd.trafo)(x, y) = camera["trafo"][x][y];
                }
            }
        }

        // xxxjack should check whether the camera with this serial already exists
        config.all_camera_configs.push_back(cd);
        camera_index++;
    }
}

void to_json(json& json_data, const RS2CaptureConfig& config) {
    json cameras;
    int camera_index = 0;

    for (RS2CameraConfig cd : config.all_camera_configs) {
        json camera;
        _MY_JSON_PUT(camera, serial, cd, serial);
        if (cd.disabled) {
            _MY_JSON_PUT(camera, disabled, cd, disabled);
        }
        if (cd.playback_filename != "") {
            _MY_JSON_PUT(camera, playback_filename, cd, playback_filename);
        }
        if (cd.type == "realsense_playback") {
            _MY_JSON_PUT(camera, playback_inpoint_micros, cd, playback_inpoint_micros);
        }
        _MY_JSON_PUT(camera, type, cd, type);

        camera["trafo"] = {
            {(*cd.trafo)(0, 0), (*cd.trafo)(0, 1), (*cd.trafo)(0, 2), (*cd.trafo)(0, 3)},
            {(*cd.trafo)(1, 0), (*cd.trafo)(1, 1), (*cd.trafo)(1, 2), (*cd.trafo)(1, 3)},
            {(*cd.trafo)(2, 0), (*cd.trafo)(2, 1), (*cd.trafo)(2, 2), (*cd.trafo)(2, 3)},
            {(*cd.trafo)(3, 0), (*cd.trafo)(3, 1), (*cd.trafo)(3, 2), (*cd.trafo)(3, 3)},
        };
        cameras[camera_index] = camera;
        camera_index++;
    }

    json_data["camera"] = cameras;
    const RS2CaptureSyncConfig& sync(config.sync);
    json sync_data;
    _MY_JSON_PUT(sync_data, sync_master_serial, sync, sync_master_serial);
    _MY_JSON_PUT(sync_data, sync_mode, sync, sync_mode);
    json_data["sync"] = sync_data;
    
    const RS2CameraProcessingParameters& filtering(config.filtering);
    json filtering_data;
    _MY_JSON_PUT(filtering_data, map_color_to_depth, filtering, map_color_to_depth);
    _MY_JSON_PUT(filtering_data, do_decimation, filtering, do_decimation);
    _MY_JSON_PUT(filtering_data, decimation_magnitude, filtering, decimation_magnitude);
    _MY_JSON_PUT(filtering_data, do_threshold, filtering, do_threshold);
    _MY_JSON_PUT(filtering_data, threshold_min_distance, filtering, threshold_min_distance);
    _MY_JSON_PUT(filtering_data, threshold_max_distance, filtering, threshold_max_distance);
    _MY_JSON_PUT(filtering_data, do_spatial, filtering, do_spatial);
    _MY_JSON_PUT(filtering_data, spatial_magnitude, filtering, spatial_magnitude);
    _MY_JSON_PUT(filtering_data, spatial_smooth_alpha, filtering, spatial_smooth_alpha);
    _MY_JSON_PUT(filtering_data, spatial_smooth_delta, filtering, spatial_smooth_delta);
    _MY_JSON_PUT(filtering_data, spatial_holes_fill, filtering, spatial_holes_fill);
    _MY_JSON_PUT(filtering_data, do_temporal, filtering, do_temporal);
    _MY_JSON_PUT(filtering_data, temporal_smooth_alpha, filtering, temporal_smooth_alpha);
    _MY_JSON_PUT(filtering_data, temporal_smooth_delta, filtering, temporal_smooth_delta);
    _MY_JSON_PUT(filtering_data, temporal_persistency, filtering, temporal_persistency);
    _MY_JSON_PUT(filtering_data, do_hole_filling, filtering, do_hole_filling);
    _MY_JSON_PUT(filtering_data, hole_filling_mode, filtering, hole_filling_mode);
    json_data["filtering"] = filtering_data;

    const RS2CaptureProcessingConfig& processing(config.processing);
    json processing_data;
    _MY_JSON_PUT(processing_data, greenscreen_removal, processing, greenscreen_removal);
    _MY_JSON_PUT(processing_data, height_min, processing, height_min);
    _MY_JSON_PUT(processing_data, height_max, processing, height_max);
    _MY_JSON_PUT(processing_data, radius_filter, processing, radius_filter);
    _MY_JSON_PUT(processing_data, depth_x_erosion, processing, depth_x_erosion);
    _MY_JSON_PUT(processing_data, depth_y_erosion, processing, depth_y_erosion);
    json_data["processing"] = processing_data;

    json system_data;
    _MY_JSON_PUT(system_data, record_to_directory, config, record_to_directory);
    _MY_JSON_PUT(system_data, playback_realtime, config, playback_realtime);
    _MY_JSON_PUT(system_data, new_timestamps, config, new_timestamps);
    _MY_JSON_PUT(system_data, debug, config, debug);
    _MY_JSON_PUT(system_data, prefer_color_timing, config, prefer_color_timing);
    json_data["system"] = system_data;
    
    const RS2CameraHardwareConfig& hardware(config.hardware);
    json hardware_data;
    _MY_JSON_PUT(hardware_data, color_width, hardware, color_width);
    _MY_JSON_PUT(hardware_data, color_height, hardware, color_height);
    _MY_JSON_PUT(hardware_data, depth_width, hardware, depth_width);
    _MY_JSON_PUT(hardware_data, depth_height, hardware, depth_height);
    _MY_JSON_PUT(hardware_data, fps, hardware, fps);
    _MY_JSON_PUT(hardware_data, visual_preset, hardware, visual_preset);
    _MY_JSON_PUT(hardware_data, color_exposure, hardware, color_exposure);
    _MY_JSON_PUT(hardware_data, color_gain, hardware, color_gain);
    _MY_JSON_PUT(hardware_data, depth_exposure, hardware, depth_exposure);
    _MY_JSON_PUT(hardware_data, depth_gain, hardware, depth_gain);
    _MY_JSON_PUT(hardware_data, whitebalance, hardware, whitebalance);
    _MY_JSON_PUT(hardware_data, backlight_compensation, hardware, backlight_compensation);
    _MY_JSON_PUT(hardware_data, laser_power, hardware, laser_power);
    json_data["hardware"] = hardware_data;

    json_data["version"] = 4;
    json_data["type"] = config.type;
}

bool cwipc_rs2_jsonfile2config(const char* filename, RS2CaptureConfig* config, std::string typeWanted) {
    json json_data;

    try {
        std::ifstream f(filename);

        if (!f.is_open()) {
            cwipc_rs2_log_warning(std::string("CameraConfig ") + filename + " not found");
            return false;
        }
        json_data = json::parse(f);

        int version = 0;
        json_data.at("version").get_to(version);

        if (version != 4) {
            cwipc_rs2_log_warning(std::string("CameraConfig ") + filename + " ignored, is not version 4");
            return false;
        }

        std::string type;
        json_data.at("type").get_to(type);

        if (type != typeWanted) {
            cwipc_rs2_log_warning(std::string("CameraConfig ") + filename + " ignored, is not " + typeWanted + " but " + type);
            return false;
        }

        from_json(json_data, *config);
    } catch (const std::exception& e) {
        cwipc_rs2_log_warning(std::string("CameraConfig ") + filename + ": exception " + e.what() );
        return false;
    }

    return true;
}

bool cwipc_rs2_jsonbuffer2config(const char* jsonBuffer, RS2CaptureConfig* config, std::string typeWanted) {
    json json_data;

    try {
        json_data = json::parse(jsonBuffer);

        int version = 0;
        json_data.at("version").get_to(version);

        if (version != 3) {
            cwipc_rs2_log_warning(std::string("CameraConfig ") + "(inline buffer) " + "ignored, is not version 3");
            return false;
        }

        std::string type;
        json_data.at("type").get_to(type);

        if (type != typeWanted) {
            cwipc_rs2_log_warning(std::string("CameraConfig ") + "(inline buffer) " + "ignored: type=" + type + " but expected " + type);
            return false;
        }

        from_json(json_data, *config);
    } catch (const std::exception& e) {
        cwipc_rs2_log_warning(std::string("CameraConfig ") + "(inline buffer) " + ": exception " + e.what() );
        return false;
    }

    json dbg_result;
    to_json(dbg_result, *config);
    std::cerr << "xxxjack debug json parse result: \n" << dbg_result << "\n";

    return true;
}

std::string cwipc_rs2_config2string(RS2CaptureConfig *config) {
    json result;
    to_json(result, *config);

    return result.dump();
}
