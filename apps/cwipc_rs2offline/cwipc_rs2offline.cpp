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
    cwipc_rs2offline_settings settings = {
		{
			640,
			480,
			3,
			60,
			CWIPC_RS2_FORMAT_RGB8
		},
		{
			640,
			480,
			4,
			60,
			CWIPC_RS2_FORMAT_Z16
		}
	};

    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " configfile colorimage depthimage outputfile" << std::endl;
		std::cerr << "Convert a depth image and color image to a pointcloud" << std::endl;
		return 2;
    }
	char *configFile = argv[1];
	char *colorFile = argv[2];
	char *depthFile = argv[3];
	char *outputFile = argv[4];
	if (strcmp(configFile, "-") == 0) configFile = NULL;

    char *error = NULL;
    cwipc_offline *converter;
	cwipc_tiledsource *generator;
	if (argc == 4) {
		configFile = argv[3];
	}
	converter = cwipc_rs2offline(settings, configFile, &error, CWIPC_API_VERSION);
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
    unsigned char *colorData = stbi_load(colorFile, &colorWidth, &colorHeight, &colorComponents, 3);
	assert(colorWidth == 640);
	assert(colorHeight == 480);
	assert(colorComponents == 3 || colorComponents == 4);
    size_t colorDataSize = colorWidth*colorHeight*3;
	int frameNum = 0;
	ok = converter->feed(0, frameNum, colorData, colorDataSize, depthData, depthDataSize);
	if (!ok) {
		std::cerr << argv[0] << ": Error feeding color and depth data" << std::endl;
		exit(1);
	}
	if(!generator->available(true)) {
		std::cerr << argv[0] << ": No pointcloud produced" << std::endl;
		exit(1);
	}
	cwipc *pc = generator->get();
	if (pc == NULL) {
		std::cerr << argv[0] << ": NULL pointcloud?" << std::endl;
		exit(1);
	}
	if (pc->get_uncompressed_size() == 0) {
		std::cerr << argv[0] << ": Empty pointcloud" << std::endl;
	} else {
		if (strcmp(outputFile, "-") != 0) {
			int sts = cwipc_write(outputFile, pc, &error);
			if (sts < 0) {
				if (error == NULL) error = (char *)"Unknown error";
				std::cerr << argv[0] << ": Error writing output file: " << error << std::endl;
			}
		}
	}
	pc->free();
    generator->free();
    return 0;
}

