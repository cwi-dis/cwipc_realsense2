//
//  pcl_renderer.cpp
//
//  Created by Fons Kuijk on 23-06-18.
//
#include <iostream>
#include <fstream>
#include <librealsense2/rs.hpp>

#include "cwipc_util/api.h"
#include "cwipc_realsense2/api.h"

void printhelp() {
	std::cout << "\nThe cloud rendered by this application will automatically be centered with the view origin.\n";
	std::cout << "To examine the pointcloud use the mouse: left-press and move mouse to move in plane, scroll wheel to move up/down\n";
	std::cout << "Use \"q\" to quit\n";
}

int main(int argc, char * argv[]) try
{
	printhelp();


	char *msg;
	cwipc_sink *viewer = cwipc_window("cwipc_renderer", &msg, CWIPC_API_VERSION);
	if (viewer == NULL) {
		std::cerr << "cwipc_renderer: ERROR: could not instantiate viewer: " << msg << std::endl;
		return EXIT_FAILURE;
	}
	cwipc_source *src = cwipc_realsense2(NULL, &msg, CWIPC_API_VERSION);
	if (src == NULL) {
		std::cerr << "cwipc_renderer: ERROR: could not instantiate realsense2 grabber: " << msg << std::endl;
		return EXIT_FAILURE;
	}
	while (true) {
		cwipc *pc = src->get();
		if (pc == NULL) {
			std::cerr << "cwipc_renderer: ERROR: NULL pointcloud" << std::endl;
			return EXIT_FAILURE;
		}
		viewer->feed(pc, true);
		pc->free();
		if (viewer->interact("Press q to quit", "q", 20) == 'q') break;

	}
	src->free();
	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "cwipc_renderer: Error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << "cwipc_renderer: " << e.what() << std::endl;
	return EXIT_FAILURE;
}
