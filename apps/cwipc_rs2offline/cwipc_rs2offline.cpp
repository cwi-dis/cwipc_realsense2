#include <iostream>
#include <fstream>
#include "string.h"

#include "cwipc_util/api.h"
#include "cwipc_realsense2/api.h"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#include "stb_image.h"

int main(int argc, char** argv)
{
    bool ok;

    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " configfile depthimage colorimage outputfile" << std::endl;
		std::cerr << "Convert a depth image and color image to a pointcloud" << std::endl;
		return 2;
    }
	char *configFile = argv[1];
	char *depthFile = argv[2];
	char *colorFile = argv[3];
	char *outputFile = argv[4];
	if (strcmp(configFile, "-") == 0) configFile = NULL;

    char *error = NULL;
    cwipc_offline *converter;
	cwipc_tiledsource *generator;
	if (argc == 4) {
		configFile = argv[3];
	}
	converter = cwipc_rs2offline(configFile, &error, CWIPC_API_VERSION);
    if (error) {
    	std::cerr << argv[0] << ": creating realsense2 offline converter failed: " << error << std::endl;
    	return 1;
    }
	generator = converter->get_source();

	int depthWidth, depthHeight, depthComponents;
    unsigned short *depthData = stbi_load_16(depthFile, &depthWidth, &depthHeight, &depthComponents, 1);
	assert(depthWidth == 640);
	assert(depthHeight == 480);
	assert(depthComponents == 1);
    size_t depthDataSize = depthWidth*depthHeight*2;
    int colorWidth, colorHeight, colorComponents;
    unsigned char *colorData = stbi_load(colorFile, &colorWidth, &colorHeight, &colorComponents, 4);
	assert(colorWidth == 640);
	assert(colorHeight == 480);
	assert(colorComponents == 4);
    size_t colorDataSize = colorWidth*colorHeight*4;

	ok = converter->feed(0, 1, colorData, colorDataSize);
	if (!ok) {
		std::cerr << argv[0] << ": Error feeding color data" << std::endl;
		return 1;
	}
	ok = converter->feed(0, 0, depthData, depthDataSize);
	if (!ok) {
		std::cerr << argv[0] << ": Error feeding depth data" << std::endl;
		return 1;
	}

	cwipc *pc = generator->get();
		if (strcmp(outputFile, "-") != 0) {
			bool ok = cwipc_write(outputFile, pc, &error);
			if (!ok) {
				std::cerr << argv[0] << ": Error writing output file: " << error << std::endl;
		}
		pc->free();
    }
    generator->free();
    return 0;
}

