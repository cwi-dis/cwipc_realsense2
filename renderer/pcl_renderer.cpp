//
//  pcl_renderer.cpp
//
//  Created by Fons Kuijk on 23-06-18.
//

#include "pcl_renderer.hpp"
#define CENTERSTEPS 256

int main(int argc, char * argv[]) try
{
	printhelp();

	GetPointCloudFunction getPointCloud = nullptr;
#ifdef WIN32
	HINSTANCE hInstLibrary;

	hInstLibrary = LoadLibrary(TEXT("multiFrame.dll"));

	if (hInstLibrary)		// the function dll file has been found and is loaded
		getPointCloud = (GetPointCloudFunction)GetProcAddress(hInstLibrary, "getPointCloud");
	else
		cerr << "ERROR: no dll file named 'multiFrame.dll' found\n";
#else
#endif // WIN32

	if (!getPointCloud)	{
		cerr << "ERROR: function 'getPointCloud' not found in dll file\n";
		return EXIT_FAILURE;
	}

	// Create a simple OpenGL window for rendering:
	window app(2560, 1440, "Multicamera Capturing");
	// Construct an object to manage view state
	glfw_state app_state;

	// register callbacks to allow manipulation of the PointCloud
	register_glfw_callbacks(app, app_state);

	int frame_num = 0;
	uint64_t time = 0;
	Eigen::Vector4f newcenter;
	Eigen::Vector4f deltacenter;


	while (app) {
		boost::shared_ptr<PointCloudT> captured_pc;
		void* pc = reinterpret_cast<void *> (&captured_pc);
		long t = 4;
		getPointCloud(&t, &pc);
		std::cout << "got a pointcloud captured at = " << t << std::endl;

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
		draw_pointcloud(app, app_state, captured_pc);
	}
#ifdef WIN32
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
