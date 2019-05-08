
#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif

#include "cwipc_util/api_pcl.h"
#include "cwipc_util/api.h"
#include "cwipc_realsense2/api.h"

#include "cwipc_realsense2/multiFrame.hpp"

class cwipc_source_realsense2_impl : public cwipc_tiledsource {
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
    
    int maxtile()
    {
        if (m_grabber == NULL) return 0;
        int nCamera = m_grabber->configuration.camera_data.size();
        if (nCamera <= 1) {
            // Using a single camera or synthetic grabber. 1 tile only.
            return 1;
        }
        return 1<<nCamera;
    }
    
    bool get_tileinfo(int tilenum, struct cwipc_tileinfo *tileinfo, int infoVersion) {
        if (m_grabber == NULL) return false;
        if (infoVersion != CWIPC_TILEINFO_VERSION)
            return false;
        int nCamera = m_grabber->configuration.camera_data.size();
        if (nCamera == 0) nCamera = 1; // The synthetic camera...
        if (tilenum < 0 || tilenum >= (1<<nCamera)) return false;
        cwipc_tileinfo info = {0, 0, 180, 180};
        switch(tilenum) {
        	case 0:
            case 1:
            case 2:
            case 4:
            case 8:
            case 16:
            case 32:
            case 64:
            case 128:
                // xxxjack fill in info from m_grabber->configuration.camera_data
                if (tileinfo) *tileinfo = info;
                return true;
        }
		return false;
    }

};

//
// C-compatible entry points
//

cwipc_tiledsource* cwipc_realsense2(char **errorMessage)
{
	return new cwipc_source_realsense2_impl();
}

cwipc_tiledsource* cwipc_realsense2_ex(const char *configFilename, char **errorMessage)
{
	return new cwipc_source_realsense2_impl(configFilename);
}
