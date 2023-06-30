//
//  utils.cpp
//
//  Created by Fons Kuijk on 12-12-18.
//
#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif
#include "cwipc_realsense2/private/RS2Config.hpp"

#include "tinyxml.h"

#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#define _MY_JSON_GET(jsonobj, name, config, attr) if (jsonobj.contains(#name)) jsonobj.at(#name).get_to(config.attr)
#define _MY_JSON_PUT(jsonobj, name, config, attr) jsonobj[#name] = config.attr

static std::string cwipc_rs2_most_recent_warning;
char **cwipc_rs2_warning_store;

void cwipc_rs2_log_warning(std::string warning)
{
    std::cerr << "cwipc_realsense2: Warning: " << warning << std::endl;
    if (cwipc_rs2_warning_store) {
        cwipc_rs2_most_recent_warning = warning;
        *cwipc_rs2_warning_store = (char *)cwipc_rs2_most_recent_warning.c_str();
    }
}

void from_json(const json& json_data, RS2CaptureConfig& config) {
    // version and type should already have been checked.
    
    json system_data = json_data.at("system");
    _MY_JSON_GET(system_data, usb2width, config, usb2_width);
    _MY_JSON_GET(system_data, usb2height, config, usb2_height);
    _MY_JSON_GET(system_data, usb2fps, config, usb2_fps);
    _MY_JSON_GET(system_data, usb3width, config, usb3_width);
    _MY_JSON_GET(system_data, usb3height, config, usb3_height);
    _MY_JSON_GET(system_data, usb3fps, config, usb3_fps);
    _MY_JSON_GET(system_data, usb2allowed, config, usb2allowed);
    _MY_JSON_GET(system_data, density_preferred, config, density);
    _MY_JSON_GET(system_data, exposure, config, exposure);
    _MY_JSON_GET(system_data, whitebalance, config, whitebalance);
    _MY_JSON_GET(system_data, backlight_compensation, config, backlight_compensation);
    _MY_JSON_GET(system_data, laser_power, config, laser_power);
    
    json postprocessing = json_data.at("postprocessing");
    _MY_JSON_GET(postprocessing, greenscreenremoval, config, greenscreen_removal);
    _MY_JSON_GET(postprocessing, height_min, config, height_min);
    _MY_JSON_GET(postprocessing, height_max, config, height_max);

    json depthfilterparameters = postprocessing.at("depthfilterparameters");
    _MY_JSON_GET(depthfilterparameters, do_decimation, config.camera_processing, do_decimation);
    _MY_JSON_GET(depthfilterparameters, decimation_value, config.camera_processing, decimation_value);
    _MY_JSON_GET(depthfilterparameters, do_threshold, config.camera_processing, do_threshold);
    _MY_JSON_GET(depthfilterparameters, threshold_near, config.camera_processing, threshold_near);
    _MY_JSON_GET(depthfilterparameters, threshold_far, config.camera_processing, threshold_far);
    _MY_JSON_GET(depthfilterparameters, do_spatial, config.camera_processing, do_spatial);
    _MY_JSON_GET(depthfilterparameters, spatial_iterations, config.camera_processing, spatial_iterations);
    _MY_JSON_GET(depthfilterparameters, spatial_alpha, config.camera_processing, spatial_alpha);
    _MY_JSON_GET(depthfilterparameters, spatial_delta, config.camera_processing, spatial_delta);
    _MY_JSON_GET(depthfilterparameters, spatial_filling, config.camera_processing, spatial_filling);
    _MY_JSON_GET(depthfilterparameters, do_temporal, config.camera_processing, do_temporal);
    _MY_JSON_GET(depthfilterparameters, temporal_alpha, config.camera_processing, temporal_alpha);
    _MY_JSON_GET(depthfilterparameters, temporal_delta, config.camera_processing, temporal_delta);
    _MY_JSON_GET(depthfilterparameters, temporal_percistency, config.camera_processing, temporal_percistency);
    
    json cameras = json_data.at("camera");
    int camera_index = 0;
    for(json::iterator it=cameras.begin(); it != cameras.end(); it++) {
        json camera = *it;
        RS2CameraConfig cd;
        pcl::shared_ptr<Eigen::Affine3d> default_trafo(new Eigen::Affine3d());
        default_trafo->setIdentity();
        cd.trafo = default_trafo;
        cd.intrinsicTrafo = default_trafo;
        _MY_JSON_GET(camera, serial, cd, serial);
        _MY_JSON_GET(camera, type, cd, type);
        if (camera.contains("trafo")) {
            for(int x=0; x<4; x++) {
                for(int y=0; y<4; y++) {
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
    json_data["cameras"] = cameras;

    json depthfilterparameters;
    _MY_JSON_PUT(depthfilterparameters, do_decimation, config.camera_processing, do_decimation);
    _MY_JSON_PUT(depthfilterparameters, decimation_value, config.camera_processing, decimation_value);
    _MY_JSON_PUT(depthfilterparameters, do_threshold, config.camera_processing, do_threshold);
    _MY_JSON_PUT(depthfilterparameters, threshold_near, config.camera_processing, threshold_near);
    _MY_JSON_PUT(depthfilterparameters, threshold_far, config.camera_processing, threshold_far);
    _MY_JSON_PUT(depthfilterparameters, do_spatial, config.camera_processing, do_spatial);
    _MY_JSON_PUT(depthfilterparameters, spatial_iterations, config.camera_processing, spatial_iterations);
    _MY_JSON_PUT(depthfilterparameters, spatial_alpha, config.camera_processing, spatial_alpha);
    _MY_JSON_PUT(depthfilterparameters, spatial_delta, config.camera_processing, spatial_delta);
    _MY_JSON_PUT(depthfilterparameters, spatial_filling, config.camera_processing, spatial_filling);
    _MY_JSON_PUT(depthfilterparameters, do_temporal, config.camera_processing, do_temporal);
    _MY_JSON_PUT(depthfilterparameters, temporal_alpha, config.camera_processing, temporal_alpha);
    _MY_JSON_PUT(depthfilterparameters, temporal_delta, config.camera_processing, temporal_delta);
    _MY_JSON_PUT(depthfilterparameters, temporal_percistency, config.camera_processing, temporal_percistency);
 
    json postprocessing;
    postprocessing["depthfilterparameters"] = depthfilterparameters;

    _MY_JSON_PUT(postprocessing, greenscreenremoval, config, greenscreen_removal);
    _MY_JSON_PUT(postprocessing, height_min, config, height_min);
    _MY_JSON_PUT(postprocessing, height_max, config, height_max);
    json_data["postprocessing"] = postprocessing;
    
    json system_data;
    _MY_JSON_PUT(system_data, usb2width, config, usb2_width);
    _MY_JSON_PUT(system_data, usb2height, config, usb2_height);
    _MY_JSON_PUT(system_data, usb2fps, config, usb2_fps);
    _MY_JSON_PUT(system_data, usb3width, config, usb3_width);
    _MY_JSON_PUT(system_data, usb3height, config, usb3_height);
    _MY_JSON_PUT(system_data, usb3fps, config, usb3_fps);
    _MY_JSON_PUT(system_data, usb2allowed, config, usb2allowed);
    _MY_JSON_PUT(system_data, density_preferred, config, density);
    _MY_JSON_PUT(system_data, exposure, config, exposure);
    _MY_JSON_PUT(system_data, whitebalance, config, whitebalance);
    _MY_JSON_PUT(system_data, backlight_compensation, config, backlight_compensation);
    _MY_JSON_PUT(system_data, laser_power, config, laser_power);
    json_data["system"] = system_data;
    
    json_data["version"] = 3;
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
        if (version != 3) {
            cwipc_rs2_log_warning(std::string("CameraConfig ") + filename + " ignored, is not version 3");
            return false;
        }
        std::string type;
        json_data.at("type").get_to(type);
        if (type != typeWanted) {
            cwipc_rs2_log_warning(std::string("CameraConfig ") + filename + " ignored, is not " + typeWanted + " but " + type);
            return false;
        }
        from_json(json_data, *config);
    }
    catch (const std::exception& e) {
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
    }
    catch (const std::exception& e) {
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

bool cwipc_rs2_xmlfile2config(const char* filename, RS2CaptureConfig* config, std::string typeWanted) {
	TiXmlDocument doc(filename);
	bool loadOkay = doc.LoadFile();
	if (!loadOkay)
	{
        cwipc_rs2_log_warning(std::string("Failed to load configfile ") + filename);
		return false;
	}

	TiXmlHandle docHandle(&doc);
	TiXmlElement* configElement = docHandle.FirstChild("file").FirstChild("CameraConfig").ToElement();
    int version = -1;
    configElement->QueryIntAttribute("version", &version);
    if (version != 2) {
        cwipc_rs2_log_warning(std::string("CameraConfig ") + filename + " is not version 2");
    }
    config->type = typeWanted;

	// get the system related information
	TiXmlElement* systemElement = configElement->FirstChildElement("system");
	if (systemElement) {
		systemElement->QueryIntAttribute("usb2width", &(config->usb2_width));
		systemElement->QueryIntAttribute("usb2height", &(config->usb2_height));
		systemElement->QueryIntAttribute("usb2fps", &(config->usb2_fps));
		systemElement->QueryIntAttribute("usb3width", &(config->usb3_width));
		systemElement->QueryIntAttribute("usb3height", &(config->usb3_height));
        systemElement->QueryIntAttribute("usb3fps", &(config->usb3_fps));
        systemElement->QueryBoolAttribute("usb2allowed", &(config->usb2allowed));
        systemElement->QueryBoolAttribute("density_preferred", &(config->density));
        systemElement->QueryIntAttribute("exposure", &(config->exposure));
        systemElement->QueryIntAttribute("whitebalance", &(config->whitebalance));
        systemElement->QueryIntAttribute("backlight_compensation", &(config->backlight_compensation));
        systemElement->QueryIntAttribute("laser_power", &(config->laser_power));
	}

    // get the processing related information
    TiXmlElement* postprocessingElement = configElement->FirstChildElement("postprocessing");
    if (postprocessingElement) {
		postprocessingElement->QueryBoolAttribute("greenscreenremoval", &(config->greenscreen_removal));
		postprocessingElement->QueryDoubleAttribute("height_min", &(config->height_min));
		postprocessingElement->QueryDoubleAttribute("height_max", &(config->height_max));

        TiXmlElement* parameterElement = postprocessingElement->FirstChildElement("depthfilterparameters");
        if (parameterElement) {
			parameterElement->QueryBoolAttribute("do_decimation", &(config->camera_processing.do_decimation));
			parameterElement->QueryIntAttribute("decimation_value", &(config->camera_processing.decimation_value));
			parameterElement->QueryBoolAttribute("do_threshold", &(config->camera_processing.do_threshold));
			parameterElement->QueryDoubleAttribute("threshold_near", &(config->camera_processing.threshold_near));
			parameterElement->QueryDoubleAttribute("threshold_far", &(config->camera_processing.threshold_far));
            parameterElement->QueryIntAttribute("depth_x_erosion", &(config->camera_processing.depth_x_erosion));
            parameterElement->QueryIntAttribute("depth_y_erosion", &(config->camera_processing.depth_y_erosion));
			parameterElement->QueryBoolAttribute("do_spatial", &(config->camera_processing.do_spatial));
			parameterElement->QueryIntAttribute("spatial_iterations", &(config->camera_processing.spatial_iterations));
			parameterElement->QueryDoubleAttribute("spatial_alpha", &(config->camera_processing.spatial_alpha));
			parameterElement->QueryIntAttribute("spatial_delta", &(config->camera_processing.spatial_delta));
			parameterElement->QueryIntAttribute("spatial_filling", &(config->camera_processing.spatial_filling));
			parameterElement->QueryBoolAttribute("do_temporal", &(config->camera_processing.do_temporal));
			parameterElement->QueryDoubleAttribute("temporal_alpha", &(config->camera_processing.temporal_alpha));
			parameterElement->QueryIntAttribute("temporal_delta", &(config->camera_processing.temporal_delta));
			parameterElement->QueryIntAttribute("temporal_percistency", &(config->camera_processing.temporal_percistency));
        }
    }
    
	bool allnewcameras = config->all_camera_configs.size() == 0; // if empty we have to set up a new administration
	int registeredcameras = 0;

	// now get the per camera info
	TiXmlElement* cameraElement = configElement->FirstChildElement("camera");
	while (cameraElement)
	{
		const char * serial = cameraElement->Attribute("serial");
		RS2CameraConfig* cd;

		int i = 0;
		while (i < config->all_camera_configs.size()) {
			if (config->all_camera_configs[i].serial == serial) {
				cameraElement->QueryBoolAttribute("disabled", &(config->all_camera_configs[i].disabled));
				cd = &config->all_camera_configs[i];
				break;
			}
			i++;
		}
		if (i == config->all_camera_configs.size()) {
			// this camera was not in the admin yet
			if (!allnewcameras)
				loadOkay = false;

			cd = new RS2CameraConfig();
			pcl::shared_ptr<Eigen::Affine3d> trafo(new Eigen::Affine3d());
			pcl::shared_ptr<Eigen::Affine3d> intrinsicTrafo(new Eigen::Affine3d());
			intrinsicTrafo->setIdentity();
			cd->serial = cameraElement->Attribute("serial");
			cameraElement->QueryBoolAttribute("disabled", &cd->disabled);
			cd->trafo = trafo;
			cd->intrinsicTrafo = intrinsicTrafo;
			cd->cameraposition = { 0, 0, 0 };
			config->all_camera_configs.push_back(*cd);
			cd = &config->all_camera_configs.back();
		}

        const char * type = cameraElement->Attribute("type");
        if (type != nullptr) {
            cd->type = type;
        }
        else {
            cd->type = config->type;
        }
        if (cd->type != config->type) {
            cwipc_rs2_log_warning(std::string("camera type mismatch: " + cd->type + " versus " + config->type));
        }
        
		TiXmlElement *trafo = cameraElement->FirstChildElement("trafo");
		if (trafo) {
			TiXmlElement *val = trafo->FirstChildElement("values");
			val->QueryDoubleAttribute("v00", &(*cd->trafo)(0, 0));
			val->QueryDoubleAttribute("v01", &(*cd->trafo)(0, 1));
			val->QueryDoubleAttribute("v02", &(*cd->trafo)(0, 2));
			val->QueryDoubleAttribute("v03", &(*cd->trafo)(0, 3));
			val->QueryDoubleAttribute("v10", &(*cd->trafo)(1, 0));
			val->QueryDoubleAttribute("v11", &(*cd->trafo)(1, 1));
			val->QueryDoubleAttribute("v12", &(*cd->trafo)(1, 2));
			val->QueryDoubleAttribute("v13", &(*cd->trafo)(1, 3));
			val->QueryDoubleAttribute("v20", &(*cd->trafo)(2, 0));
			val->QueryDoubleAttribute("v21", &(*cd->trafo)(2, 1));
			val->QueryDoubleAttribute("v22", &(*cd->trafo)(2, 2));
			val->QueryDoubleAttribute("v23", &(*cd->trafo)(2, 3));
			val->QueryDoubleAttribute("v30", &(*cd->trafo)(3, 0));
			val->QueryDoubleAttribute("v31", &(*cd->trafo)(3, 1));
			val->QueryDoubleAttribute("v32", &(*cd->trafo)(3, 2));
			val->QueryDoubleAttribute("v33", &(*cd->trafo)(3, 3));
		}
		else
			loadOkay = false;
		// load optional intrinsicTrafo element (only for offline usage)
		trafo = cameraElement->FirstChildElement("intrinsicTrafo");
		if (trafo) {
            if (cd->intrinsicTrafo == nullptr) {
                pcl::shared_ptr<Eigen::Affine3d> intrinsic_trafo(new Eigen::Affine3d());
                cd->intrinsicTrafo = intrinsic_trafo;
            }
			TiXmlElement *val = trafo->FirstChildElement("values");
			val->QueryDoubleAttribute("v00", &(*cd->intrinsicTrafo)(0, 0));
			val->QueryDoubleAttribute("v01", &(*cd->intrinsicTrafo)(0, 1));
			val->QueryDoubleAttribute("v02", &(*cd->intrinsicTrafo)(0, 2));
			val->QueryDoubleAttribute("v03", &(*cd->intrinsicTrafo)(0, 3));
			val->QueryDoubleAttribute("v10", &(*cd->intrinsicTrafo)(1, 0));
			val->QueryDoubleAttribute("v11", &(*cd->intrinsicTrafo)(1, 1));
			val->QueryDoubleAttribute("v12", &(*cd->intrinsicTrafo)(1, 2));
			val->QueryDoubleAttribute("v13", &(*cd->intrinsicTrafo)(1, 3));
			val->QueryDoubleAttribute("v20", &(*cd->intrinsicTrafo)(2, 0));
			val->QueryDoubleAttribute("v21", &(*cd->intrinsicTrafo)(2, 1));
			val->QueryDoubleAttribute("v22", &(*cd->intrinsicTrafo)(2, 2));
			val->QueryDoubleAttribute("v23", &(*cd->intrinsicTrafo)(2, 3));
			val->QueryDoubleAttribute("v30", &(*cd->intrinsicTrafo)(3, 0));
			val->QueryDoubleAttribute("v31", &(*cd->intrinsicTrafo)(3, 1));
			val->QueryDoubleAttribute("v32", &(*cd->intrinsicTrafo)(3, 2));
			val->QueryDoubleAttribute("v33", &(*cd->intrinsicTrafo)(3, 3));
		}

		registeredcameras++;
		cameraElement = cameraElement->NextSiblingElement("camera");
	}
	if (config->all_camera_configs.size() != registeredcameras)
		loadOkay = false;

    if (!loadOkay) {
        cwipc_rs2_log_warning("Available hardware camera configuration does not match configuration file");
    }
	return loadOkay;
}
