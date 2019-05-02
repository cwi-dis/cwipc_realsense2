#include <iostream>
#include <fstream>

#include "cwipc_util/api.h"
#include "cwipc_realsense2/api.h"

int main(int argc, char** argv)
{
    //char *message = NULL;
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " count directory [configfile]" << std::endl;
        std::cerr << "Creates COUNT pointclouds from a realsense2 camera and stores the PLY files in the given DIRECTORY" << std::endl;
        return 2;
    }
    int count = atoi(argv[1]);
    char filename[500];
    char *error = NULL;
    
	cwipc_source *generator;
	if (argc == 3) {
		generator = cwipc_realsense2(&error);
	}
	else {
		generator = cwipc_realsense2_ex(argv[3], &error);
	}
    if (error) {
    	std::cerr << argv[0] << ": creating realsense2 grabber failed: " << error << std::endl;
    	return 1;
    }
    int ok = 0;
    while (count-- > 0 && ok == 0) {
    	cwipc *pc = generator->get();
    	snprintf(filename, sizeof(filename), "%s/pointcloud-%lld.ply", argv[2], pc->timestamp());
    	ok = cwipc_write(filename, pc, &error);
    }
    if (ok < 0) {
    	std::cerr << "Error: " << error << std::endl;
    	return 1;
    }
    return 0;
}

