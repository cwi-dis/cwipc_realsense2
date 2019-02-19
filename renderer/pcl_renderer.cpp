//
//  pcl_renderer.cpp
//
//  Created by Fons Kuijk on 23-06-18.
//

#include "window_util.hpp"
#include "cwipc_realsense/defs.h"
#include <librealsense2/rs.hpp>

#ifdef WITH_WIN32_LOADLIBRARY
#include <windows.h>
#else
#include "cwipc_realsense/api.h"
#endif

typedef void(*GetPointCloudFunction)(uint64_t *, void **);

#define CENTERSTEPS 256

bool do_align = false;
bool rotation = true;
int aligncamera = 0;
Eigen::Vector4f mergedcenter;	// needed to automatically center the merged cloud

void printhelp() {
	cout << "\nThe cloud rendered by this application will automatically be centered with the view origin.\n";
	cout << "To examine the pointcloud use the mouse: leftclick and move to rotate, use the mouse wheel to zoom.\n";
	cout << "Use \"esc\" to reset the position of the (fused) cloud.\n";
	cout << "Use \"q\" to quit\n";
}

// Handle the OpenGL setup needed to display all pointclouds
void draw_pointcloud(window_util* app, boost::shared_ptr<PointCloudT> point_cloud)
{
	app->prepare_gl(-mergedcenter.x(), -mergedcenter.y(), -mergedcenter.z());

	// draw the pointcloud(s)
	for (auto pnt : point_cloud->points) {
		float col[] = { (float)pnt.r / 256.f, (float)pnt.g / 256.f, (float)pnt.b / 256.f };
		glColor3fv(col);
		float vert[] = { pnt.x, pnt.y, pnt.z };
		glVertex3fv(vert);
	}

	app->cleanup_gl();
}

// Registers the state variable and callbacks to allow mouse control of the pointcloud
void register_glfw_callbacks(window_util& app)
{
	app.on_left_mouse = [&](bool pressed) {
		app.app_state()->ml = pressed;
	};

	app.on_mouse_scroll = [&](double xoffset, double yoffset) {
		app.app_state()->offset -= static_cast<float>(yoffset);
	};

	app.on_mouse_move = [&](double x, double y) {
		if (app.app_state()->ml) {

			app.app_state()->yaw += (x - app.app_state()->last_x) / 10.0;
			app.app_state()->pitch += (y - app.app_state()->last_y) / 10.0;
			app.app_state()->pitch = std::max(app.app_state()->pitch, -85.0);
			app.app_state()->pitch = std::min(app.app_state()->pitch, +85.0);
		}
		app.app_state()->last_x = x;
		app.app_state()->last_y = y;
	};

	app.on_key_release = [&](int key) {
		if (key == 256) { // Escape is interpreted as a reset of the transformation

			app.app_state()->yaw = app.app_state()->pitch = 0;
			app.app_state()->offset = 0.0;
		}
		else if (key == 81) {	// key = "q": Quit program
			exit(0);
		}
	};
}

int main(int argc, char * argv[]) try
{
	printhelp();

#ifdef WITH_WIN32_LOADLIBRARY
	GetPointCloudFunction getPointCloud = nullptr;
	HINSTANCE hInstLibrary;

	hInstLibrary = LoadLibrary(TEXT("multiFrame.dll"));

	if (hInstLibrary)		// the function dll file has been found and is loaded
		getPointCloud = (GetPointCloudFunction)GetProcAddress(hInstLibrary, "getPointCloud");
	else
		cerr << "ERROR: no dll file named 'multiFrame.dll' found\n";
	if (!getPointCloud)	{
		cerr << "ERROR: function 'getPointCloud' not found in dll file\n";
		return EXIT_FAILURE;
	}
#else
#endif // WITH_WIN32_LOADLIBRARY


	// Create a simple OpenGL window for rendering:
	window_util app(2560, 1440, "Multicamera Capturing");

	// register callbacks to allow manipulation of the PointCloud
	register_glfw_callbacks(app);

	int frame_num = 0;
	uint64_t time = 0;
	Eigen::Vector4f newcenter;
	Eigen::Vector4f deltacenter;


	while (app) {
		boost::shared_ptr<PointCloudT> captured_pc;
		void* pc = reinterpret_cast<void *> (&captured_pc);
		uint64_t t = 4;
		getPointCloud(&t, &pc);

		captured_pc = *reinterpret_cast<boost::shared_ptr<PointCloudT>*>(pc);
		
		if (!(captured_pc.get()->size() > 0)) continue;

		// Automatically centre the cloud
		if (!(frame_num++ % CENTERSTEPS)) {
			pcl::compute3DCentroid(*captured_pc, newcenter);
			deltacenter = (newcenter - mergedcenter) / CENTERSTEPS;
		}
		if (!do_align)
			mergedcenter += deltacenter;

		// NB: draw pointcloud ignores the obtained pointcloud, as it may want to draw individual pointclouds rather than the merged one.
		draw_pointcloud(&app, captured_pc);
	}
#ifdef WITH_WIN32_LOADLIBRARY
	FreeLibrary(hInstLibrary);
#endif
	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	cerr << "Error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
