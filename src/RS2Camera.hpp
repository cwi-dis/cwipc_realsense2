#ifndef cwipc_realsense_RS2Camera_hpp
#define cwipc_realsense_RS2Camera_hpp
#pragma once

#include "RS2BaseCamera.hpp"

class RS2Camera : public RS2BaseCamera {
public:
    using RS2BaseCamera::RS2BaseCamera;
    
    virtual void post_start_all_cameras() override;
    virtual bool seek(uint64_t timestamp) override;
protected:
    virtual void _init_config_for_this_camera(rs2::config &cfg) override final;
    virtual void _post_start(rs2::pipeline_profile& profile) override final;
};
#endif // cwipc_realsense_RS2Camera_hpp
