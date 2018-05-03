//
//  multiFrame.hpp
//  src
//
//  Created by Fons Kuijk on 23-04-18.
//

#ifndef multiFrame_hpp
#define multiFrame_hpp

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <atomic>
#include <stdio.h>
#include <vector>
#include <map>
#include <iostream>

struct rs_frame {
    rs2::video_frame* color;
    rs2::points* points;
};

class multiFrame {
    
public:
    multiFrame(int cameras) {
        frame_pool.resize(cameras);
    }
    
    int register_camera(std::string serialnumber) {
        //      if (numCams < frame_pool.size() && registrations.find(serialnumber) == registrations.end()) {
        //           registrations[serialnumber] = numCams;
        return numCams++;
        //        }
        //        return -1;
    }
    
    void push_frame(rs2::points* points, const int camNum)
    {
        if (camNum < 0 || camNum > frame_pool.size()) return;
        std::cout << "push: p " << points << ", cam " << camNum << "\n";
        if (points == NULL)  return;
        frame_pool[camNum].points = points;
    }
    
    rs2::points* pull_frame(int camNum = -1)
    {
        if (camNum < 0 || camNum > frame_pool.size())
            return NULL;
        else
            return frame_pool[camNum].points;
    }
    
    void push_color_frame(rs2::video_frame* color, const int camNum)
    {
        if (camNum < 0 || camNum > frame_pool.size()) return;
        std::cout << "push: c " << color << ", cam " << camNum << "\n";
        if (color == NULL)  return;
        frame_pool[camNum].color = color;
    }
    
    rs2::video_frame* pull_color_frame(int camNum = -1)
    {
        if (camNum < 0 || camNum > frame_pool.size())
            return NULL;
        else
            return frame_pool[camNum].color;
    }
    
    void fuse_frames();
    
    __SIZE_TYPE__ nframes() { return frame_pool.size(); }
    
private:
    std::vector<rs_frame> frame_pool;
    std::map<std::string, int> registrations;
    rs2::points fusedFrame;
    int numCams = 0;
};


#endif /* multiFrame_hpp */
