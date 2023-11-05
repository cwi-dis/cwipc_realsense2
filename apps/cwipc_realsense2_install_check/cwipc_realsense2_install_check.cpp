#include <iostream>
#include <fstream>
#include "string.h"
#include <stdlib.h>
#include <inttypes.h>
#include "cwipc_util/api.h"
#include "cwipc_realsense2/api.h"

int main(int argc, char** argv) {
    char *error = NULL;
    cwipc_source *generator = cwipc_realsense2("auto", &error, CWIPC_API_VERSION);
    if (generator == NULL) {
        char *expectedError = strstr(error, "no realsense cameras found");
        if (expectedError == NULL) {
            // Any other error is unexpected.
            std::cerr << argv[0] << ": Error: " << error << std::endl;
            return 1;
        }
    }
    return 0;
}

