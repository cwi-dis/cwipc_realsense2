//
//  multiFrame.cpp
//  src
//
//  Created by Fons Kuijk on 23-04-18.
//

#include "multiFrame.hpp"

void multiFrame::fuse_frames()
{
    int fusedFrameSize = 0;
    const int nViews = frame_pool.size();
    for (int cNum = 0; cNum < nViews; cNum++) {
        //fusedFrameSize += sizeof(frame_pool[cNum]->depth);
    }
    /*
    std::vector<int[3]> pixels(fusedFrameSize);
    std::vector<float[3]> pc_points(fusedFrameSize);
    int pointCounter = 0;
    for (int cNum = 0; cNum < nViews; cNum++) {
        pc_my_frame framePart = *frame_pool[cNum];
        for (int pNum = 0; pNum < sizeof(frame_pool[cNum]->depth); pNum++) {
            pc_points[pointCounter][0] = framePart.points[pNum][0];
            pc_points[pointCounter][1] = framePart.points[pNum][1];
            pc_points[pointCounter][2] = framePart.points[pNum][2];
            pixels[pointCounter][0] = framePart.colors[pNum][0];
            pixels[pointCounter][1] = framePart.colors[pNum][1];
            pixels[pointCounter][2] = framePart.colors[pNum][2];
        }
    }
    fusedFrame.depth = pc_points;
    fusedFrame.color = pixels;
    */
    return;
}

