//
//  pcl_renderer.cpp
//
//  Created by Fons Kuijk on 23-06-18.
//
#include "pcl_renderer.hpp"

int main(int argc, char * argv[]) try
{
	// Create a simple OpenGL window for rendering:
	window app(1280, 720, "RealSense Multicamera Capturing");
	// Construct an object to manage view state
	glfw_state app_state;
	// Construct a capturing object
	multiFrame multiframe;

	// register callbacks to allow manipulation of the PointCloud
	register_glfw_callbacks(app, app_state, multiframe);

	frameNum = 0;
	while (app) {
		uint64_t time = 0;
		boost::shared_ptr<PointCloud<PointXYZRGB>> captured_pc;
		void* pc = reinterpret_cast<void *> (&captured_pc);

		// Here we call the capture software
		multiframe.get_pointcloud(&time, &pc);

		captured_pc = *reinterpret_cast<boost::shared_ptr<PointCloud<PointXYZRGB>>*>(pc);

		if (captured_pc.get() == NULL) continue;

		// Write a ply file of the PointCloud
		//cloud2file(captured_pc);

		// Draw the PointCloud
		draw_pointcloud(app, app_state, captured_pc);
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
