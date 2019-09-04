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
bool mf_file2config(const char* filename, MFCaptureConfig* config)
{
	TiXmlDocument doc(filename);
	bool loadOkay = doc.LoadFile();
	if (!loadOkay)
	{
		std::cerr << "cwipc_realsense2: multiFrame: Warning: Failed to load configfile " << filename << "\n";
		if (config->cameraData.size() > 1)
			std::cerr << "cwipc_realsense2: multiFrame: Warning: Captured pointclouds will be merged based on unregistered cameras\n";
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
		postprocessingElement->QueryDoubleAttribute("tilingresolution", &(config->tiling_resolution));
		const char* method = postprocessingElement->Attribute("tilingmethod");
		if (method) config->tiling_method.assign(method);
        
        TiXmlElement* parameterElement = postprocessingElement->FirstChildElement("depthfilterparameters");
        if (parameterElement) {
			parameterElement->QueryIntAttribute("decimation_value", &(config->default_camera_settings.decimation_value));
			parameterElement->QueryDoubleAttribute("threshold_near", &(config->default_camera_settings.threshold_near));
			parameterElement->QueryDoubleAttribute("threshold_far", &(config->default_camera_settings.threshold_far));
			parameterElement->QueryIntAttribute("spatial_iterations", &(config->default_camera_settings.spatial_iterations));
			parameterElement->QueryDoubleAttribute("spatial_alpha", &(config->default_camera_settings.spatial_alpha));
			parameterElement->QueryIntAttribute("spatial_delta", &(config->default_camera_settings.spatial_delta));
			parameterElement->QueryIntAttribute("spatial_filling", &(config->default_camera_settings.spatial_filling));
			parameterElement->QueryDoubleAttribute("temporal_alpha", &(config->default_camera_settings.temporal_alpha));
			parameterElement->QueryIntAttribute("temporal_delta", &(config->default_camera_settings.temporal_delta));
			parameterElement->QueryIntAttribute("temporal_percistency", &(config->default_camera_settings.temporal_percistency));
        }
    }
    
	bool allnewcameras = config->cameraData.size() == 0; // if empty we have to set up a new administration
	int registeredcameras = 0;

	// now get the per camera info
	TiXmlElement* cameraElement = configElement->FirstChildElement("camera");
	while (cameraElement)
	{
		const char * serial = cameraElement->Attribute("serial");
		MFCameraData* cd;

		int i = 0;
		while (i < config->cameraData.size()) {
			if (config->cameraData[i].serial == serial) {
				cd = &config->cameraData[i];
				break;
			}
			i++;
		}
		if (i == config->cameraData.size()) {
			// this camera was not in the admin yet
			if (!allnewcameras)
				loadOkay = false;

			cd = new MFCameraData();
			boost::shared_ptr<Eigen::Affine3d> trafo(new Eigen::Affine3d());
			boost::shared_ptr<Eigen::Affine3d> intrinsicTrafo(new Eigen::Affine3d());
			intrinsicTrafo->setIdentity();
			cd->serial = cameraElement->Attribute("serial");
			cd->trafo = trafo;
			cd->intrinsicTrafo = intrinsicTrafo;
			cd->background = { 0, 0, 0 };
			cd->cameraposition = { 0, 0, 0 };
			config->cameraData.push_back(*cd);
			cd = &config->cameraData.back();
		}
		cameraElement->QueryDoubleAttribute("backgroundx", &(cd->background.x));
		cameraElement->QueryDoubleAttribute("backgroundy", &(cd->background.y));
		cameraElement->QueryDoubleAttribute("backgroundz", &(cd->background.z));

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
	if (config->cameraData.size() != registeredcameras)
		loadOkay = false;

	if (!loadOkay)
		std::cerr << "cwipc_realsense2: multiFrame: Warning: the configuration file specifying " << registeredcameras
		<< " cameras did not correspond to the setup of " << config->cameraData.size()
		<< " cameras\n\tre-alignment may be needed!!\n";

	return loadOkay;
}

// store the current camera transformation setting into a xml document
void mf_config2file(const char* filename, MFCaptureConfig* config)
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
	postprocessing->SetDoubleAttribute("tilingresolution", config->tiling_resolution);
	postprocessing->SetAttribute("tilingmethod", config->tiling_method.c_str());
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
	parameters->SetAttribute("decimation_value", config->default_camera_settings.decimation_value);
	parameters->SetDoubleAttribute("threshold_near", config->default_camera_settings.threshold_near);
	parameters->SetDoubleAttribute("threshold_far", config->default_camera_settings.threshold_far);
	parameters->SetAttribute("spatial_iterations", config->default_camera_settings.spatial_iterations);
	parameters->SetDoubleAttribute("spatial_alpha", config->default_camera_settings.spatial_alpha);
	parameters->SetAttribute("spatial_delta", config->default_camera_settings.spatial_delta);
	parameters->SetAttribute("spatial_filling", config->default_camera_settings.spatial_filling);
	parameters->SetDoubleAttribute("temporal_alpha", config->default_camera_settings.temporal_alpha);
	parameters->SetAttribute("temporal_delta", config->default_camera_settings.temporal_delta);
	parameters->SetAttribute("temporal_percistency", config->default_camera_settings.temporal_percistency);
	postprocessing->LinkEndChild(parameters);

	cameraconfig->LinkEndChild(new TiXmlComment(" backgroundx, backgroundy and backgroudz if not 0 position the camera's background plane "));
	for (MFCameraData cd : config->cameraData) {
		TiXmlElement* cam = new TiXmlElement("camera");
		cam->SetAttribute("serial", cd.serial.c_str());
		cam->SetDoubleAttribute("backgroundx", cd.background.x);
		cam->SetDoubleAttribute("backgroundy", cd.background.y);
		cam->SetDoubleAttribute("backgroundz", cd.background.z);
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

bool mf_noChromaRemoval(cwipc_pcl_point* p)
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
