//
//  pcl_align.cpp
//
//  Created by Fons Kuijk on 23-06-18.
//

#include "pcl_align.hpp"
#define CENTERSTEPS 256

int main(int argc, char * argv[]) try
{
	// Create a simple OpenGL window for rendering:
	window_util app(2560, 1440, "RealSense Multicamera Capturing");
	// Construct an object to manage view state
	glfw_state app_state;
	// Construct a capturing object
	multiFrame multiframe;

	// register callbacks to allow manipulation of the PointCloud
	register_glfw_callbacks(app, app_state, multiframe);

	int frame_num = 0;
	uint64_t time = 0;
	Eigen::Vector4f newcenter;
	Eigen::Vector4f deltacenter;

	printhelp();

	if (multiframe.getNumberOfCameras() < 1) {
		if (load_data() && CamData.size() > 0)
			do_align = true;
		else {
			cerr << "\nSorry: No cameras connected and no data to load\n\n";
			return EXIT_FAILURE;
		}
	}
	
	while (app) {
		if (!do_align) {
			boost::shared_ptr<PointCloudT> captured_pc;
			void* pc = reinterpret_cast<void *> (&captured_pc);

			// Here we ask for a pointcloud (the merger of all camera's) and thereby trigger the actual capturing
			multiframe.get_pointcloud(&time, &pc);

			captured_pc = *reinterpret_cast<boost::shared_ptr<PointCloudT>*>(pc);

			if (captured_pc.get() == NULL) continue;

			// Automatically centre the cloud
			if (!(frame_num++ % CENTERSTEPS)) {
				pcl::compute3DCentroid(*captured_pc, newcenter);
				deltacenter = (newcenter - mergedcenter) / CENTERSTEPS;
			}
			mergedcenter += deltacenter;
		}
		// NB: draw pointcloud ignores the just obtained pointcloud, as it may want to draw pointclouds of the camera's individually rather than the merged one.
		draw_pointcloud(app, app_state, multiframe);
	}
	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	cerr << "Error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
