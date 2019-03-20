//
//  utils.cpp
//
//  Created by Fons Kuijk on 12-12-18.
//
#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif
#include "cwipc_realsense2/defs.h"
#include "cwipc_realsense2/utils.h"

#include "tinyxml.h"

typedef struct HsvColor
{
	unsigned char h;
	unsigned char s;
	unsigned char v;
} HsvColor;

// read and restore the camera transformation setting as stored in the configuration document
bool file2config(const char* filename, configdata* config)
{
	TiXmlDocument doc(filename);
	bool loadOkay = doc.LoadFile();
	if (!loadOkay)
	{
		std::cout << "\nWARNING: Failed to load cameraconfig.xml\n";
		if (config->camera_data.size() > 1)
			std::cout << "\t Captured pointclouds will be merged based on unregistered cameras\n";
		return false;
	}

	TiXmlHandle docHandle(&doc);
	TiXmlElement* configElement = docHandle.FirstChild("file").FirstChild("CameraConfig").ToElement();

	// get the system related information
	TiXmlElement* systemElement = configElement->FirstChildElement("system");
	if (systemElement) {
		systemElement->QueryIntAttribute("usb2width", &(config->usb2_width));
		systemElement->QueryIntAttribute("usb2height", &(config->usb2_height));
		systemElement->QueryIntAttribute("usb2fps", &(config->usb2_fps));
		systemElement->QueryIntAttribute("usb3width", &(config->usb3_width));
		systemElement->QueryIntAttribute("usb3height", &(config->usb3_height));
		systemElement->QueryIntAttribute("usb3fps", &(config->usb3_fps));
	}

    // get the processing related information
    TiXmlElement* postprocessingElement = configElement->FirstChildElement("postprocessing");
    if (postprocessingElement) {
		postprocessingElement->QueryBoolAttribute("depthfiltering", &(config->depth_filtering));
		postprocessingElement->QueryBoolAttribute("backgroundremoval", &(config->background_removal));
		postprocessingElement->QueryBoolAttribute("greenscreenremoval", &(config->greenscreen_removal));
		postprocessingElement->QueryDoubleAttribute("cloudresolution", &(config->cloud_resolution));
		postprocessingElement->QueryBoolAttribute("tiling", &(config->tiling));
		postprocessingElement->QueryDoubleAttribute("tileresolution", &(config->tile_resolution));
        
        TiXmlElement* parameterElement = postprocessingElement->FirstChildElement("depthfilterparameters");
        if (parameterElement) {
			parameterElement->QueryIntAttribute("decimation_value", &(config->decimation_value));
			parameterElement->QueryIntAttribute("spatial_iterations", &(config->spatial_iterations));
			parameterElement->QueryDoubleAttribute("spatial_alpha", &(config->spatial_alpha));
			parameterElement->QueryIntAttribute("spatial_delta", &(config->spatial_delta));
			parameterElement->QueryIntAttribute("spatial_filling", &(config->spatial_filling));
			parameterElement->QueryDoubleAttribute("temporal_alpha", &(config->temporal_alpha));
			parameterElement->QueryIntAttribute("temporal_delta", &(config->temporal_delta));
			parameterElement->QueryIntAttribute("temporal_percistency", &(config->temporal_percistency));
        }
    }
    
	bool allnewcameras = config->camera_data.size() == 0; // if empty we have to set up a new administration
	int registeredcameras = 0;

	// now get the per camera info
	TiXmlElement* cameraElement = configElement->FirstChildElement("camera");
	while (cameraElement)
	{
		const char * serial = cameraElement->Attribute("serial");
		cameradata* cd;

		int i = 0;
		while (i < config->camera_data.size()) {
			if (config->camera_data[i].serial == serial) {
				cd = &config->camera_data[i];
				break;
			}
			i++;
		}
		if (i == config->camera_data.size()) {
			// this camera was not in the admin yet
			if (!allnewcameras)
				loadOkay = false;

			cd = new cameradata();
			cwipc_pcl_pointcloud empty_pntcld(new_cwipc_pcl_pointcloud());
			boost::shared_ptr<Eigen::Affine3d> trafo(new Eigen::Affine3d());
			cd->serial = cameraElement->Attribute("serial");
			cd->cloud = empty_pntcld;
			cd->trafo = trafo;
			config->camera_data.push_back(*cd);
			cd = &config->camera_data.back();
		}
		cameraElement->QueryDoubleAttribute("backgroundx", &(cd->background_x));
		cameraElement->QueryDoubleAttribute("backgroundy", &(cd->background_y));
		cameraElement->QueryDoubleAttribute("backgroundz", &(cd->background_z));

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

		registeredcameras++;
		cameraElement = cameraElement->NextSiblingElement("camera");
	}
	if (config->camera_data.size() != registeredcameras)
		loadOkay = false;

	if (!loadOkay)
		std::cout << "\nWARNING: the configuration file specifying " << registeredcameras
		<< " cameras did not correspond to the setup of " << config->camera_data.size()
		<< " cameras\n\tre-alignment may be needed!!\n";

	return loadOkay;
}

// store the current camera transformation setting into a xml document
void config2file(const char* filename, configdata* config)
{
	TiXmlDocument doc;
	doc.LinkEndChild(new TiXmlDeclaration("1.0", "", ""));

	TiXmlElement* root = new TiXmlElement("file");
	doc.LinkEndChild(root);

	TiXmlElement* cameraconfig = new TiXmlElement("CameraConfig");
	root->LinkEndChild(cameraconfig);

	TiXmlElement* system = new TiXmlElement("system");
	system->SetAttribute("usb2width", config->usb2_width);
	system->SetAttribute("usb2height", config->usb2_height);
	system->SetAttribute("usb2fps", config->usb2_fps);
	system->SetAttribute("usb3width", config->usb3_width);
	system->SetAttribute("usb3height", config->usb3_height);
	system->SetAttribute("usb3fps", config->usb3_fps);
	cameraconfig->LinkEndChild(system);

	cameraconfig->LinkEndChild(new TiXmlComment(" 'cloudresolution' and 'tileresolution' are specified in meters "));
	TiXmlElement* postprocessing = new TiXmlElement("postprocessing");
	postprocessing->SetAttribute("depthfiltering", config->depth_filtering);
	postprocessing->SetAttribute("backgroundremoval", config->background_removal);
	postprocessing->SetAttribute("greenscreenremoval", config->greenscreen_removal);
	postprocessing->SetDoubleAttribute("cloudresolution", config->cloud_resolution);
	postprocessing->SetAttribute("tiling", config->tiling);
	postprocessing->SetDoubleAttribute("tileresolution", config->tile_resolution);
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
	parameters->SetAttribute("decimation_value", config->decimation_value);
	parameters->SetAttribute("spatial_iterations", config->spatial_iterations);
	parameters->SetDoubleAttribute("spatial_alpha", config->spatial_alpha);
	parameters->SetAttribute("spatial_delta", config->spatial_delta);
	parameters->SetAttribute("spatial_filling", config->spatial_filling);
	parameters->SetDoubleAttribute("temporal_alpha", config->temporal_alpha);
	parameters->SetAttribute("temporal_delta", config->temporal_delta);
	parameters->SetAttribute("temporal_percistency", config->temporal_percistency);
	postprocessing->LinkEndChild(parameters);

	cameraconfig->LinkEndChild(new TiXmlComment(" backgroundx, backgroundy and backgroudz if not 0 position the camera's background plane "));
	for (cameradata cd : config->camera_data) {
		TiXmlElement* cam = new TiXmlElement("camera");
		cam->SetAttribute("serial", cd.serial.c_str());
		cam->SetDoubleAttribute("backgroundx", cd.background_x);
		cam->SetDoubleAttribute("backgroundy", cd.background_y);
		cam->SetDoubleAttribute("backgroundz", cd.background_z);
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

cwipc_pcl_point* hsvToRgb(HsvColor hsv, cwipc_pcl_point* pnt)
{
	unsigned char region, p, q, t;
	unsigned int h, s, v, remainder;

	if (hsv.s == 0)
	{
		pnt->r = hsv.v;
		pnt->g = hsv.v;
		pnt->b = hsv.v;
		return pnt;
	}

	// converting to 16 bit to prevent overflow
	h = hsv.h;
	s = hsv.s;
	v = hsv.v;

	region = h / 43;
	remainder = (h - (region * 43)) * 6;

	p = (v * (255 - s)) >> 8;
	q = (v * (255 - ((s * remainder) >> 8))) >> 8;
	t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

	switch (region)
	{
	case 0:
		pnt->r = v;
		pnt->g = t;
		pnt->b = p;
		break;
	case 1:
		pnt->r = q;
		pnt->g = v;
		pnt->b = p;
		break;
	case 2:
		pnt->r = p;
		pnt->g = v;
		pnt->b = t;
		break;
	case 3:
		pnt->r = p;
		pnt->g = q;
		pnt->b = v;
		break;
	case 4:
		pnt->r = t;
		pnt->g = p;
		pnt->b = v;
		break;
	default:
		pnt->r = v;
		pnt->g = p;
		pnt->b = q;
		break;
	}

	return pnt;
}

HsvColor rgbToHsv(cwipc_pcl_point* pnt)
{
	HsvColor hsv;
	unsigned char rgbMin, rgbMax;

	rgbMin = pnt->r < pnt->g ? (pnt->r < pnt->b ? pnt->r : pnt->b) : (pnt->g < pnt->b ? pnt->g : pnt->b);
	rgbMax = pnt->r > pnt->g ? (pnt->r > pnt->b ? pnt->r : pnt->b) : (pnt->g > pnt->b ? pnt->g : pnt->b);

	hsv.v = rgbMax;
	if (hsv.v == 0)
	{
		hsv.h = 0;
		hsv.s = 0;
		return hsv;
	}

	hsv.s = 255 * ((long)(rgbMax - rgbMin)) / hsv.v;
	if (hsv.s == 0)
	{
		hsv.h = 0;
		return hsv;
	}

	if (rgbMax == pnt->r)
		hsv.h = 0 + 43 * (pnt->g - pnt->b) / (rgbMax - rgbMin);
	else if (rgbMax == pnt->g)
		hsv.h = 85 + 43 * (pnt->b - pnt->r) / (rgbMax - rgbMin);
	else
		hsv.h = 171 + 43 * (pnt->r - pnt->g) / (rgbMax - rgbMin);

	return hsv;
}

bool noChromaRemoval(cwipc_pcl_point* p)
{
	HsvColor hsv = rgbToHsv(p);

	if (hsv.h >= 60 && hsv.h <= 130) {
		if (hsv.s >= 0.15 && hsv.v >= 0.15) {
			// reducegreen
			if ((p->r * p->b) != 0 && (p->g * p->g) / (p->r * p->b) > 1.5) {
				p->r *= 1.4;
				p->b *= 1.4;
			}
			else {
				p->r *= 1.2;
				p->b *= 1.2;
			}
		}
		return !(hsv.s >= 0.4 && hsv.v >= 0.3);
	}
	return true;
}
