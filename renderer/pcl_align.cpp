//
//  pcl_align.cpp
//
//  Created by Fons Kuijk on 23-06-18.
//

#include "cwipc_realsense/window_util.hpp"
#include "cwipc_realsense/multiFrame.hpp"
#include "cwipc_realsense/utils.h"

#include <string>
#include <sstream>
#include <pcl/io/ply_io.h>

#define CENTERSTEPS 256

bool do_align = false;
bool loaded_mode = false;
bool rotation = true;
int aligncamera = 0;
Eigen::Vector4f mergedcenter;	// Needed to automatically center the merged cloud
Eigen::Vector4f cloudcenter;		// Needed to be able to rotate around the cloud's centre of mass
configdata ConfigCopy;			// Still copy of the configuration data of multiFrame
string ext(".ply");

void printhelp() {
	cout << "\nThe cloud rendered by this application will automatically be centered with the view origin.\n";
	cout << "To examine the pointcloud use the mouse: leftclick and move to rotate, use the mouse wheel to zoom.\n";
	cout << "Use \"esc\" to reset the position of the (fused) cloud.\n";

	cout << "\nAction keys for alignment of camera clouds:\n";
	cout << "\t\"a\": toggle between 'life' and 'alignment mode'\n";
	cout << "\t\"1-9\": select the camera to align\n";
	cout << "\t\"r\": start cloud rotate mode\n";
	cout << "\t\"t\": start cloud translate mode\n";
	cout << "\t\"esc\": reset the cloud transformation of the active camera\n";
	cout << "\t\"s\": save the configuration and snapshots of each camera to files\n";
	cout << "\t\"l\": toggle between 'life' and a 'loaded' configuration and snapshots to (re)align\n";
	cout << "\t\"h\": print this help\n";
	cout << "\t\"q\": quit program\n";
}

void cloud2file(boost::shared_ptr<PointCloudT> pntcld, string filename)
{
	if (!pntcld) return;
	int size = pntcld->size();
	if (size <= 0) return;

	std::ofstream myfile(filename.c_str());
	myfile << "ply\n" << "format ascii 1.0\nelement vertex " << size << "\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";

	std::ostringstream oss;
	for (int i = 0; i < size; i++) {
		oss << (
			std::to_string((*pntcld)[i].x) + " " +
			std::to_string((*pntcld)[i].y) + " " +
			std::to_string((*pntcld)[i].z) + " " +
			std::to_string((*pntcld)[i].r) + " " +
			std::to_string((*pntcld)[i].g) + " " +
			std::to_string((*pntcld)[i].b) + "\n").c_str();
	}
	myfile << oss.str();
	myfile.close();
}

void makeFreezeCopy(configdata* conf)
{
	ConfigCopy = *conf;
}

bool load_ply_of_camera(cameradata camera)
{
	// Load the cloud and save it into the global list of models
	if (pcl::io::loadPLYFile<PointT>(camera.serial + ext, *camera.cloud) == -1)
	{
		string msg = "Error loading cloud from file " + camera.serial + ext + "\n";
		PCL_ERROR(msg.c_str());
		return false;
	}
	return true;
}

// Ignore camera's that may be active, load a configuration and captured frames from file
bool load_data() {
	if (!file2config("cameraconfig.xml", &ConfigCopy))
		return false;
	for (auto camera : ConfigCopy.camera_data)
		if (!load_ply_of_camera(camera)) {
			cerr << "Could not load a .ply file for camera " << camera.serial << " as specified in the configuration file\n";
			return false;
		}
	return true;
}

// Handle the OpenGL setup needed to display all pointclouds
void draw_pointcloud(window_util* app, multiFrame& multiframe)
{
	app->prepare_gl(-mergedcenter.x(), -mergedcenter.y(), -mergedcenter.z());

	// draw the pointcloud(s)
	if (do_align) {
		// draw the individual pointclouds of the still
		for (int i = 0; i < ConfigCopy.camera_data.size(); i++) {
			PointCloudT::Ptr pcptr(new PointCloudT);
			transformPointCloud(*(ConfigCopy.camera_data[i].cloud.get()), *pcptr, *ConfigCopy.camera_data[i].trafo);
			for (auto pnt : pcptr->points) {
				float col[3];
				if (i == aligncamera) {	// highlight the cloud of the selected camera
					col[0] = 0.2 + pnt.r / 320.0;
					col[1] = 0.2 + pnt.g / 320.0;
					col[2] = 0.2 + pnt.b / 320.0;
				}
				else {
					col[0] = pnt.r / 320.0;
					col[1] = pnt.g / 320.0;
					col[2] = pnt.b / 320.0;
				}
				glColor3fv(col);
				float vert[] = { pnt.x, pnt.y, pnt.z };
				glVertex3fv(vert);
			}
		}
	}
	else {
		// 'life' mode action
		if (!loaded_mode && multiframe.Configuration.camera_data.size() > 0) {
			// this is the real 'life' rendering
			for (auto pnt : multiframe.getPointCloud()->points) {
				float col[] = { (float)pnt.r / 256.f, (float)pnt.g / 256.f, (float)pnt.b / 256.f };
				glColor3fv(col);
				float vert[] = { pnt.x, pnt.y, pnt.z };
				glVertex3fv(vert);
			}
		}
		else {
			// this is the stilled 'life' rendering (loaded mode)
			for (int i = 0; i < ConfigCopy.camera_data.size(); i++) {
				PointCloudT::Ptr pcptr(new PointCloudT);
				transformPointCloud(*(ConfigCopy.camera_data[i].cloud.get()), *pcptr, *ConfigCopy.camera_data[i].trafo);
				for (auto pnt : pcptr->points) {
					float col[] = { (float)pnt.r / 256.f, (float)pnt.g / 256.f, (float)pnt.b / 256.f };
					glColor3fv(col);
					float vert[] = { pnt.x, pnt.y, pnt.z };
					glVertex3fv(vert);
				}
			}
		}
	}

	app->cleanup_gl();
}

// Registers the state variable and callbacks to allow mouse control of the pointcloud
void register_glfw_callbacks(window_util& app, multiFrame& multiframe)
{
	app.on_left_mouse = [&](bool pressed) {
		app.app_state()->ml = pressed;
	};

	app.on_mouse_scroll = [&](double xoffset, double yoffset) {
		if (do_align) {
			Eigen::Affine3d *transform = ConfigCopy.camera_data[aligncamera].trafo.get();
			if (rotation) {
				(*transform).rotate(Eigen::AngleAxisd(yoffset / 100.0, Eigen::Vector3d::UnitZ()));
			}
			else {
				(*transform).translate(Eigen::Vector3d(0.0, 0.0, -yoffset / 100.0));
			}
		}
		else {
			app.app_state()->offset -= static_cast<float>(yoffset);
		}
	};

	app.on_mouse_move = [&](double x, double y) {
		if (app.app_state()->ml) {
			if (do_align) {
				Eigen::Affine3d *transform = ConfigCopy.camera_data[aligncamera].trafo.get();
				double dx = (x - app.app_state()->last_x) / (0.25 * app.width());
				double dy = -(y - app.app_state()->last_y) / (0.25 * app.width());
				if (rotation) {
					(*transform).translate(Eigen::Vector3d(cloudcenter.x(), cloudcenter.y(), cloudcenter.z()));
					(*transform).rotate(Eigen::AngleAxisd(dx, Eigen::Vector3d::UnitY()));
					(*transform).rotate(Eigen::AngleAxisd(-dy, Eigen::Vector3d::UnitX()));
					(*transform).translate(-Eigen::Vector3d(cloudcenter.x(), cloudcenter.y(), cloudcenter.z()));
				}
				else {
					(*transform).translate(Eigen::Vector3d(dx, dy, 0.0));
				}
			}
			else {
				app.app_state()->yaw += (x - app.app_state()->last_x) / 10.0;
				app.app_state()->pitch += (y - app.app_state()->last_y) / 10.0;
				app.app_state()->pitch = std::max(app.app_state()->pitch, -85.0);
				app.app_state()->pitch = std::min(app.app_state()->pitch, +85.0);
			}
		}
		app.app_state()->last_x = x;
		app.app_state()->last_y = y;
	};

	app.on_key_release = [&](int key) {
		if (key == 256) { // Escape is interpreted as a reset of the transformation
			if (do_align) {
				Eigen::Affine3d *transform = ConfigCopy.camera_data[aligncamera].trafo.get();
				(*transform).setIdentity();
			}
			else {
				app.app_state()->yaw = app.app_state()->pitch = 0;
				app.app_state()->offset = 0.0;
			}
		}
		else if (key == 65) {	// key = "a": start/stop Alignment
			if (do_align) {	// switch to 'life'
				//if (!loaded_mode && multiframe.Configuration.camera_data.size() > 0)
				//	multiframe.Configuration = ConfigCopy; // set possible new transforms
				do_align = false;
			}
			else {			// switch to 'align'
				if (!loaded_mode && multiframe.Configuration.camera_data.size() > 0)
					makeFreezeCopy(&multiframe.Configuration); // make a still copy of multiFrame's configuration
				if (aligncamera > ConfigCopy.camera_data.size())
					aligncamera = 0;
				pcl::compute3DCentroid(*ConfigCopy.camera_data[aligncamera].cloud, cloudcenter);
				do_align = true;
			}
		}
		else if (key == 72) {	// key = "h": print help
			printhelp();
		}
		else if (key == 76) {	// key = "l": load previous result
			if (loaded_mode) {
				// leaving loaded mode
				if (multiframe.Configuration.camera_data.size() > 0) {
					makeFreezeCopy(&multiframe.Configuration); // make a still copy of multiFrame's configuration
					do_align = false; // that is the to be expected mode
				}
				loaded_mode = false;
				cout << "loaded mode switched off\n";
			}
			else {
				// starting loaded mode
				if (load_data() && ConfigCopy.camera_data.size() > 0)
					do_align = true; // that is the to be expected mode
				else {
					cerr << "\nError: Data could not be loaded\n";
					return;
				}
				if (aligncamera > ConfigCopy.camera_data.size())
					aligncamera = 0;
				pcl::compute3DCentroid(*ConfigCopy.camera_data[aligncamera].cloud, cloudcenter);
				loaded_mode = true;
				cout << "loaded mode switched on\n";
			}
		}
		else if (key == 81) {	// key = "q": Quit program
			multiframe.~multiFrame();
			exit(0);
		}
		else if (key == 82) {	// key = "r": Rotate
			rotation = true;
		}
		else if (key == 83) {	// key = "s": Save config and snapshots to file
			config2file("cameraconfig.xml", &ConfigCopy);
			for (int i = 0; i < ConfigCopy.camera_data.size(); i++)
				cloud2file(ConfigCopy.camera_data[i].cloud, ConfigCopy.camera_data[i].serial + ".ply");
		}
		else if (key == 84) {	// key = "t": Translate
			rotation = false;
		}
		else if (key >= 49 && key < ConfigCopy.camera_data.size() + 49) {	// key = "1-9": select a camera
			aligncamera = key - 49;
			pcl::compute3DCentroid(*ConfigCopy.camera_data[aligncamera].cloud, cloudcenter);
		}
		else if (key == 73) {	// key =\"i": dump frames for icp processing
			for (int i = 0; i < ConfigCopy.camera_data.size(); i++) {
				PointCloudT::Ptr point_cloud_ptr(new PointCloudT);
				boost::shared_ptr<PointCloudT> aligned_cld(point_cloud_ptr);

				transformPointCloud(*ConfigCopy.camera_data[i].cloud.get(), *aligned_cld, *ConfigCopy.camera_data[i].trafo);

				cloud2file(aligned_cld, "pcl_aligned_" + ConfigCopy.camera_data[i].serial + ".ply");
				cloud2file(ConfigCopy.camera_data[i].cloud, "pcl_original_" + ConfigCopy.camera_data[i].serial + ".ply");
			}
		}
	};
}


int main(int argc, char * argv[]) try
{
	// Create a simple OpenGL window for rendering:
	window_util app(2560, 1440, "RealSense Multicamera Capturing");
	// Construct a capturing object
	multiFrame multiframe;

	// register callbacks to allow manipulation of the PointCloud
	register_glfw_callbacks(app, multiframe);

	int frame_num = 0;
	uint64_t time = 0;
	Eigen::Vector4f newcenter;
	Eigen::Vector4f deltacenter;

	printhelp();

	if (multiframe.Configuration.camera_data.size() < 1) {
		if (load_data() && ConfigCopy.camera_data.size() > 0)
			do_align = true;
		else {
			std::cerr << "\nSorry: No cameras connected and no data to load\n\n";
			return EXIT_FAILURE;
		}
	}
	else
		makeFreezeCopy(&multiframe.Configuration); // make a still copy of multiFrame's configuration

	while (app) {
		if (!(do_align || loaded_mode)) {
			boost::shared_ptr<PointCloudT> captured_pc;
			void* pc = reinterpret_cast<void *> (&captured_pc);

			// Here we ask for a pointcloud (the merger of all camera's) and thereby trigger the actual capturing
			cwipc_pcl_pointcloud captured_pc = multiframe.get_pointcloud(&time);

			if (captured_pc.get() == NULL) continue;

			// Automatically centre the cloud
			if (!(frame_num++ % CENTERSTEPS)) {
				pcl::compute3DCentroid(*captured_pc, newcenter);
				deltacenter = (newcenter - mergedcenter) / CENTERSTEPS;
			}
			mergedcenter += deltacenter;
		}
		// NB: draw pointcloud ignores the just obtained pointcloud, as it may want to draw pointclouds of the camera's individually rather than the merged one.
		draw_pointcloud(&app, multiframe);
	}
	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "Error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
