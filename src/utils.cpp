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
#include "cwipc_realsense2/multiFrame.hpp"

#include "tinyxml.h"

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
		systemElement->QueryUnsignedAttribute("ringbuffersize", &(config->ringbuffer_size));
		config->ringbuffer_size = config->ringbuffer_size < 1 ? 1 : config->ringbuffer_size;
	}

	// get the processing related information
	TiXmlElement* processingElement = configElement->FirstChildElement("processing");
	if (processingElement) {
		processingElement->QueryBoolAttribute("backgroundremoval", &(config->background_removal));
		processingElement->QueryBoolAttribute("greenscreenremoval", &(config->greenscreen_removal));
		processingElement->QueryBoolAttribute("tiling", &(config->tiling));
		processingElement->QueryDoubleAttribute("cloudresolution", &(config->cloud_resolution));
		processingElement->QueryDoubleAttribute("tileresolution", &(config->tile_resolution));
	}

	bool allnewcameras = config->camera_data.size() == 0; // we have to set up a new administration
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
	system->SetAttribute("ringbuffersize", config->ringbuffer_size);
	cameraconfig->LinkEndChild(system);

	TiXmlElement* processing = new TiXmlElement("processing");
	processing->SetAttribute("backgroundremoval", config->background_removal);
	processing->SetAttribute("greenscreenremoval", config->greenscreen_removal);
	processing->SetAttribute("tiling", config->tiling);
	processing->SetDoubleAttribute("cloudresolution", config->cloud_resolution);
	processing->SetDoubleAttribute("tileresolution", config->tile_resolution);
	cameraconfig->LinkEndChild(processing);

	for (cameradata cd : config->camera_data) {
		TiXmlElement* cam = new TiXmlElement("camera");
		cam->SetAttribute("serial", cd.serial.c_str());
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
