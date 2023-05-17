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

void from_json(const json& j, RS2CaptureConfig* config) {
    
}

bool cwipc_rs2_jsonfile2config(const char* filename, RS2CaptureConfig* config) {
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
            cwipc_rs2_log_warning(std::string("CameraConfig ") + filename + " is not version 3");
        }
        from_json(json_data, config);
    }
    catch (const std::exception& e) {
        cwipc_rs2_log_warning(std::string("CameraConfig ") + filename + ": exception " + e.what() );
        return false;
    }
	return true;
}

bool cwipc_rs2_xmlfile2config(const char* filename, RS2CaptureConfig* config) {
	TiXmlDocument doc(filename);
	bool loadOkay = doc.LoadFile();
	if (!loadOkay)
	{
        cwipc_rs2_log_warning(std::string("Failed to load configfile ") + filename + ", using default matrices");
		return false;
	}

	TiXmlHandle docHandle(&doc);
	TiXmlElement* configElement = docHandle.FirstChild("file").FirstChild("CameraConfig").ToElement();
    int version = -1;
    configElement->QueryIntAttribute("version", &version);
    if (version != 2) {
        cwipc_rs2_log_warning(std::string("CameraConfig ") + filename + " is not version 2");
    }

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
			parameterElement->QueryBoolAttribute("do_decimation", &(config->camera_config.do_decimation));
			parameterElement->QueryIntAttribute("decimation_value", &(config->camera_config.decimation_value));
			parameterElement->QueryBoolAttribute("do_threshold", &(config->camera_config.do_threshold));
			parameterElement->QueryDoubleAttribute("threshold_near", &(config->camera_config.threshold_near));
			parameterElement->QueryDoubleAttribute("threshold_far", &(config->camera_config.threshold_far));
            parameterElement->QueryIntAttribute("depth_x_erosion", &(config->camera_config.depth_x_erosion));
            parameterElement->QueryIntAttribute("depth_y_erosion", &(config->camera_config.depth_y_erosion));
			parameterElement->QueryBoolAttribute("do_spatial", &(config->camera_config.do_spatial));
			parameterElement->QueryIntAttribute("spatial_iterations", &(config->camera_config.spatial_iterations));
			parameterElement->QueryDoubleAttribute("spatial_alpha", &(config->camera_config.spatial_alpha));
			parameterElement->QueryIntAttribute("spatial_delta", &(config->camera_config.spatial_delta));
			parameterElement->QueryIntAttribute("spatial_filling", &(config->camera_config.spatial_filling));
			parameterElement->QueryBoolAttribute("do_temporal", &(config->camera_config.do_temporal));
			parameterElement->QueryDoubleAttribute("temporal_alpha", &(config->camera_config.temporal_alpha));
			parameterElement->QueryIntAttribute("temporal_delta", &(config->camera_config.temporal_delta));
			parameterElement->QueryIntAttribute("temporal_percistency", &(config->camera_config.temporal_percistency));
        }
    }
    
	bool allnewcameras = config->camera_data.size() == 0; // if empty we have to set up a new administration
	int registeredcameras = 0;

	// now get the per camera info
	TiXmlElement* cameraElement = configElement->FirstChildElement("camera");
	while (cameraElement)
	{
		const char * serial = cameraElement->Attribute("serial");
		RS2CameraData* cd;

		int i = 0;
		while (i < config->camera_data.size()) {
			if (config->camera_data[i].serial == serial) {
				cameraElement->QueryBoolAttribute("disabled", &(config->camera_data[i].disabled));
				cd = &config->camera_data[i];
				break;
			}
			i++;
		}
		if (i == config->camera_data.size()) {
			// this camera was not in the admin yet
			if (!allnewcameras)
				loadOkay = false;

			cd = new RS2CameraData();
			pcl::shared_ptr<Eigen::Affine3d> trafo(new Eigen::Affine3d());
			pcl::shared_ptr<Eigen::Affine3d> intrinsicTrafo(new Eigen::Affine3d());
			intrinsicTrafo->setIdentity();
			cd->serial = cameraElement->Attribute("serial");
			cameraElement->QueryBoolAttribute("disabled", &cd->disabled);
			cd->trafo = trafo;
			cd->intrinsicTrafo = intrinsicTrafo;
			cd->cameraposition = { 0, 0, 0 };
			config->camera_data.push_back(*cd);
			cd = &config->camera_data.back();
		}

        const char *type = cameraElement->Attribute("type");
        if (type != NULL && *type != '\0') {
            cd->type = type;
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
	if (config->camera_data.size() != registeredcameras)
		loadOkay = false;

    if (!loadOkay) {
        cwipc_rs2_log_warning("Available hardware camera configuration does not match configuration file");
    }
	return loadOkay;
}

// read and restore the camera transformation setting as stored in the configuration document
bool cwipc_rs2_file2config(const char* filename, RS2CaptureConfig* config)
{
	std::string sFilename(filename);
	std::cerr << "xxxjack filename=" << sFilename << std::endl;
	if (sFilename.substr(sFilename.find_last_of(".")+1) == "json") {
		return cwipc_rs2_jsonfile2config(filename, config);
	} else {
		return cwipc_rs2_xmlfile2config(filename, config);
	}
}

// store the current camera transformation setting into a xml document
void cwipc_rs2_config2file(const char* filename, RS2CaptureConfig* config)
{
	TiXmlDocument doc;
	doc.LinkEndChild(new TiXmlDeclaration("1.0", "", ""));

	TiXmlElement* root = new TiXmlElement("file");
	doc.LinkEndChild(root);

	TiXmlElement* cameraconfig = new TiXmlElement("CameraConfig");
    cameraconfig->SetAttribute("version", 2);
    root->LinkEndChild(cameraconfig);

	TiXmlElement* system = new TiXmlElement("system");
	system->SetAttribute("usb2width", config->usb2_width);
	system->SetAttribute("usb2height", config->usb2_height);
	system->SetAttribute("usb2fps", config->usb2_fps);
	system->SetAttribute("usb3width", config->usb3_width);
	system->SetAttribute("usb3height", config->usb3_height);
    system->SetAttribute("usb3fps", config->usb3_fps);
    system->SetAttribute("usb2allowed", config->usb2allowed);
    system->SetAttribute("exposure", config->exposure);
    system->SetAttribute("whitebalance", config->whitebalance);
    system->SetAttribute("backlight_compensation", config->backlight_compensation);
    system->SetAttribute("laser_power", config->laser_power);
    system->SetAttribute("density_preferred", config->density);
	cameraconfig->LinkEndChild(system);

	cameraconfig->LinkEndChild(new TiXmlComment(" 'cloudresolution' and 'tileresolution' are specified in meters "));
	TiXmlElement* postprocessing = new TiXmlElement("postprocessing");
	postprocessing->SetAttribute("greenscreenremoval", config->greenscreen_removal);
	postprocessing->SetDoubleAttribute("height_min", config->height_min);
	postprocessing->SetDoubleAttribute("height_max", config->height_max);
	cameraconfig->LinkEndChild(postprocessing);

	postprocessing->LinkEndChild(new TiXmlComment(" For information on depth filtering parameters see librealsense/doc/post-processing-filters.md "));
	postprocessing->LinkEndChild(new TiXmlComment("\tdecimation_value is an int between 2 and 8 "));
	postprocessing->LinkEndChild(new TiXmlComment("\tspatial_iterations is an int between 1 and 5 "));
	postprocessing->LinkEndChild(new TiXmlComment("\tspatial_alpha is is a float between 0.25 and 1.0 "));
	postprocessing->LinkEndChild(new TiXmlComment("\tspatial_delta is an int between 1 and 50 "));
	postprocessing->LinkEndChild(new TiXmlComment("\tspatial_filling is an int between 0 and 6 "));
	postprocessing->LinkEndChild(new TiXmlComment("\ttemporal_alpha is is a float between 0 and 1 "));
	postprocessing->LinkEndChild(new TiXmlComment("\ttemporal_delta is is an int between 1 and 100 "));
	postprocessing->LinkEndChild(new TiXmlComment("\ttemporal_percistency is a float between 0 and 8 "));

	TiXmlElement* parameters = new TiXmlElement("depthfilterparameters");
	parameters->SetAttribute("do_decimation", config->camera_config.do_decimation);
	parameters->SetAttribute("decimation_value", config->camera_config.decimation_value);
	parameters->SetAttribute("do_threshold", config->camera_config.do_threshold);
	parameters->SetDoubleAttribute("threshold_near", config->camera_config.threshold_near);
	parameters->SetDoubleAttribute("threshold_far", config->camera_config.threshold_far);
	parameters->SetAttribute("do_spatial", config->camera_config.do_spatial);
	parameters->SetAttribute("spatial_iterations", config->camera_config.spatial_iterations);
	parameters->SetDoubleAttribute("spatial_alpha", config->camera_config.spatial_alpha);
	parameters->SetAttribute("spatial_delta", config->camera_config.spatial_delta);
	parameters->SetAttribute("spatial_filling", config->camera_config.spatial_filling);
	parameters->SetAttribute("do_temporal", config->camera_config.do_temporal);
	parameters->SetDoubleAttribute("temporal_alpha", config->camera_config.temporal_alpha);
	parameters->SetAttribute("temporal_delta", config->camera_config.temporal_delta);
	parameters->SetAttribute("temporal_percistency", config->camera_config.temporal_percistency);
	postprocessing->LinkEndChild(parameters);

	cameraconfig->LinkEndChild(new TiXmlComment(" backgroundx, backgroundy and backgroudz if not 0 position the camera's background plane "));
	for (RS2CameraData cd : config->camera_data) {
		TiXmlElement* cam = new TiXmlElement("camera");
		cam->SetAttribute("serial", cd.serial.c_str());
        cam->SetAttribute("type", cd.type.c_str());
		cameraconfig->LinkEndChild(cam);

		TiXmlElement* trafo = new TiXmlElement("trafo");
		cam->LinkEndChild(trafo);

		TiXmlElement* val = new TiXmlElement("values");
		val->SetDoubleAttribute("v00", (*cd.trafo)(0, 0));
		val->SetDoubleAttribute("v01", (*cd.trafo)(0, 1));
		val->SetDoubleAttribute("v02", (*cd.trafo)(0, 2));
		val->SetDoubleAttribute("v03", (*cd.trafo)(0, 3));
		val->SetDoubleAttribute("v10", (*cd.trafo)(1, 0));
		val->SetDoubleAttribute("v11", (*cd.trafo)(1, 1));
		val->SetDoubleAttribute("v12", (*cd.trafo)(1, 2));
		val->SetDoubleAttribute("v13", (*cd.trafo)(1, 3));
		val->SetDoubleAttribute("v20", (*cd.trafo)(2, 0));
		val->SetDoubleAttribute("v21", (*cd.trafo)(2, 1));
		val->SetDoubleAttribute("v22", (*cd.trafo)(2, 2));
		val->SetDoubleAttribute("v23", (*cd.trafo)(2, 3));
		val->SetDoubleAttribute("v30", (*cd.trafo)(3, 0));
		val->SetDoubleAttribute("v31", (*cd.trafo)(3, 1));
		val->SetDoubleAttribute("v32", (*cd.trafo)(3, 2));
		val->SetDoubleAttribute("v33", (*cd.trafo)(3, 3));
		trafo->LinkEndChild(val);
	}
	doc.SaveFile(filename);
}

