//
//  multiFrame.hpp
//  src
//
//  Created by Fons Kuijk on 23-04-18.
//

#ifndef multiFrame_hpp
#define multiFrame_hpp

#include <atomic>
#include <stdio.h>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <mutex>
#include <algorithm>

#include <librealsense2/rs.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
/*
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/time.h> /**/

struct rs_frame {
    rs2::video_frame* color;
    rs2::points* points;
};

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

const int color_width = 1280;
const int color_height = 720;
const int color_fps = 30;
const int depth_width = 1280;
const int depth_height = 720;
const int depth_fps = 30;


class multiFrame {
    std::mutex frames_mutex;

public:
    multiFrame() {
        rs2::context ctx;    // Create librealsense context for managing devices
        auto devs = ctx.query_devices();
        numberOfCameras = devs.size();
        frame_pool.resize(numberOfCameras);
        if (numberOfCameras == 0) {
 /*        // no cameras connected, then read testfile instead
            PointCloudT::Ptr cloud_in(new PointCloudT);
            std::string testfile = "right.ply";
            if (pcl::io::loadPLYFile<pcl::PointXYZ>(testfile, *cloud_in) == -1)
            {
                PCL_ERROR("Error loading cloud %s.\n");
            }
            std::cout << "No cameras found, loaded" << cloud_in->size() << "data points from testfile: " << testfile << std::endl;
/**/
        }

        // Start capturing for all connected RealSense devices
        for (int dNum = 0; dNum < numberOfCameras; dNum++) {
            std::string serial_number(devs[dNum].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
            //if (dNum == 1) serial_number = "802212060048";
            //if (dNum == 0) serial_number = "746112061432";
            capture_start(serial_number, this, dNum);
        }
    }
    
    // Configure and initialize caputuring of one camera
    void capture_start(const std::string& serial_number, multiFrame* m_frame, int camNum)
    {
        std::thread([serial_number, m_frame, camNum]() mutable {
            
            std::cout << "starting camera ser no: " << serial_number << '\n';
            rs2::config cfg;
            cfg.enable_device(serial_number);
            cfg.enable_stream(RS2_STREAM_COLOR, color_width, color_height, RS2_FORMAT_RGB8, color_fps);
            cfg.enable_stream(RS2_STREAM_DEPTH, depth_width, depth_height, RS2_FORMAT_Z16, depth_fps);
            
            // Declare RealSense pipeline, encapsulating the actual device and sensors
            rs2::pipeline pipe;
            
            // Start streaming with the configuration just set
            pipe.start(cfg);
            
            // Declare pointcloud object, for calculating pointclouds and texture mappings
            rs2::pointcloud pc;
            // We want the points object to be persistent so we can display the last cloud when a frame drops
            rs2::points points;
            
            while (true) // Application still alive?
            {
                // Wait for the next set of frames from the camera
                auto frames = pipe.wait_for_frames();
                auto depth = frames.get_depth_frame();
                auto color = frames.get_color_frame();
                float min = 100.f, max;

                // exclude pushing frames at this time
                std::lock_guard<std::mutex> guard(m_frame->frames_mutex);
                
                // Tell points frame to map to this color frame
                pc.map_to(color); // does not align the frames. That is handled by setting resolution of cameras

                points = pc.calculate(depth);
                std::cout << "timestamp = " << points.get_timestamp() << "\n";
                
                // Push the frames for live rendering, not needed to generate pointclouds!
                m_frame->push_color_frame(&color, camNum);
                m_frame->push_frame(&points, camNum);
                
                // Generate new vertices and color vector
                auto vertices = points.get_vertices();
                //points.export_to_ply("frame.ply", color);

                unsigned char *p = (unsigned char*)color.get_data();

                // Calculate the closest point distance
                for (int i = 0; i < points.size(); i++)
                    if (vertices[i].z != 0 && min > vertices[i].z)
                        min = vertices[i].z;
                
                // Set the maximum distance
                max = 0.8f + min;
                
                // Make PointCloud
                m_frame->pointcloud.clear();
                PointCloudT partial_cloud;
                for (int i = 0; i < points.size(); i++)
                {
                    if (min < vertices[i].z && vertices[i].z < max) // Skip background
                    {
                        pcl::PointXYZRGB pt;
                        pt.x = vertices[i].x;
                        pt.y =-vertices[i].y;
                        pt.z =-vertices[i].z;
                        int pi = i*3;
                        pt.r = p[pi];
                        pt.g = p[pi+1];
                        pt.b = p[pi+2];
                        m_frame->pointcloud.push_back(pt);
                        partial_cloud.push_back(pt);
                    }
                }
                PointCloudT transformed_cloud;
                Eigen::Affine3f transform = Eigen::Affine3f::Identity();
                transform.translation() << 0.5, 0.0, 0.0;
                transform.rotate (Eigen::AngleAxisf (1.4 , Eigen::Vector3f::UnitY()));
                pcl::transformPointCloud (partial_cloud, transformed_cloud, transform);
                
                m_frame->full_cloud = transformed_cloud;
            }
            std::cout << "stopping camera ser no: " << serial_number << '\n';
            pipe.stop();
        }
        ).detach();
    }
    
    std::vector<PointT> pull_pointcloud()
    {
        std::lock_guard<std::mutex> guard(frames_mutex);

        return pointcloud;
    }
    
    PointCloudT getPointCloud()
    {
        std::lock_guard<std::mutex> guard(frames_mutex);
        return full_cloud;
    }
    
    void push_frame(rs2::points* points, const int camNum)
    {
        if (camNum < 0 || camNum > numberOfCameras) return;
//        std::cout << "push: p " << points << ", cam " << camNum << "\n";
        if (points == NULL)  return;
        frame_pool[camNum].points = points;
    }

    rs2::points* pull_frame(int camNum = -1)
    {
        // exclude thread to work on frames
        std::lock_guard<std::mutex> guard(frames_mutex);

        if (camNum < 0 || camNum > numberOfCameras)
            return NULL;
        else
            return frame_pool[camNum].points;
    }
    
    void push_color_frame(rs2::video_frame* color, const int camNum)
    {
        if (camNum < 0 || camNum > numberOfCameras) return;
//        std::cout << "push: c " << color << ", cam " << camNum << "\n";
        if (color == NULL)  return;
        frame_pool[camNum].color = color;
    }
    
    rs2::video_frame* pull_color_frame(int camNum = -1)
    {
        // exclude thread to work on frames
        std::lock_guard<std::mutex> guard(frames_mutex);

        if (camNum < 0 || camNum > numberOfCameras)
            return NULL;
        else
            return frame_pool[camNum].color;
    }
    
    void fuse_frames();
    
    int n_cameras() { return numberOfCameras; }
       
private:
    std::vector<rs_frame> frame_pool;
    std::vector<PointT> pointcloud;
    PointCloudT full_cloud;
    int numberOfCameras = 0;
};


#endif /* multiFrame_hpp */
