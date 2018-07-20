//
//  pcl_renderer.cpp
//
//  Created by Fons Kuijk on 23-06-18.
//
#include "pcl_renderer.hpp"

int frameNum;

void cloud2file(boost::shared_ptr<PointCloud<PointXYZRGB> > pntcld)
{
	int size = pntcld->size();
	if (size <= 0) return;

	std::ofstream myfile(("pcl_frame" + std::to_string(frameNum++) + ".ply").c_str());
	myfile << "ply\n" << "format ascii 1.0\nelement vertex " << size << "\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";

	std::ostringstream oss;
	for (int i = 0; i < size; i++) {
		oss << (std::to_string((*pntcld)[i].x) + " " +
			std::to_string((*pntcld)[i].y) + " " +
			std::to_string((*pntcld)[i].z) + " " +
			std::to_string((*pntcld)[i].r) + " " +
			std::to_string((*pntcld)[i].g) + " " +
			std::to_string((*pntcld)[i].b) + "\n").c_str();
	}
	myfile << oss.str();
	myfile.close();
}

int main(int argc, char * argv[]) try
{
	// Create a simple OpenGL window for rendering:
	window app(1280, 720, "RealSense Multicamera Capturing");
	// Construct an object to manage view state
	glfw_state app_state;
	// register callbacks to allow manipulation of the PointCloud
	register_glfw_callbacks(app, app_state);

	multiFrame multiframe;

	frameNum = 0;
	while (app) {

		long time = 0;
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
