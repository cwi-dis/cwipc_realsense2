
#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif

#include "cwipc_util/api_pcl.h"
#include "cwipc_util/api.h"
#include "cwipc_realsense2/api.h"

#include "cwipc_realsense2/multiFrame.hpp"

class cwipc_source_realsense2_impl : public cwipc_source {
private:
    multiFrame *m_grabber;
public:
    cwipc_source_realsense2_impl(const char *configFilename=NULL)
		: m_grabber(NULL)
	{ 
		m_grabber = new multiFrame(configFilename); 
	}

    ~cwipc_source_realsense2_impl()
	{
        delete m_grabber;
    }

    void free() 
	{
        delete m_grabber;
    }

    bool eof() 
	{
    	return false;
    }

    bool available(bool wait)
	{
    	return m_grabber != NULL;
    }

    cwipc* get()
	{
        if (m_grabber == NULL) return NULL;
        uint64_t timestamp;
        cwipc_pcl_pointcloud pc = m_grabber->get_pointcloud(&timestamp);
        if (pc == NULL) return NULL;
        return cwipc_from_pcl(pc, timestamp, NULL);
    }
};

//
// C-compatible entry points
//

cwipc_source* cwipc_realsense2(char **errorMessage)
{
	return new cwipc_source_realsense2_impl();
}

cwipc_source* cwipc_realsense2_ex(const char *configFilename, char **errorMessage)
{
	return new cwipc_source_realsense2_impl(configFilename);
}
