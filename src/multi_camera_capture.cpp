// License: Apache 2.0. See LICENSE file in root directory.

#include "multiFrame.hpp"
#include "example.hpp"          // Include short list of convenience functions for rendering
#include <thread>               // std::thread
#include <chrono>
#include <mutex>
#include <algorithm>            // std::min, std::max

// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);
const int color_width = 1920;
const int color_height = 1080;
const int color_fps = 30;
const int depth_width = 1280;
const int depth_height = 720;
const int depth_fps = 30;
bool capturingactive = false;
std::mutex frames_mutex;

// Handle the OpenGL calls needed to display one pointcloud
void drawit(float width, float height, glfw_state& app_state, rs2::points& points)
{
    glPointSize(width / 640);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, app_state.tex.get_gl_handle());
    float tex_border_color[] = { 0.8f, 0.8f, 0.8f, 0.8f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, tex_border_color);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, 0x812F); // GL_CLAMP_TO_EDGE
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, 0x812F); // GL_CLAMP_TO_EDGE
    glBegin(GL_POINTS);
    
    /* this segment actually prints the pointcloud */
    auto vertices = points.get_vertices();              // get vertices
    auto tex_coords = points.get_texture_coordinates(); // and texture coordinates
    for (int i = 0; i < points.size(); i++)
    {
        if (vertices[i].z)
        {
            // upload the point and texture coordinates only for points we have depth data for
            glVertex3fv(vertices[i]);
            glTexCoord2fv(tex_coords[i]);
        }
    }
}

// Handle the OpenGL setup needed to display all pointclouds
void draw_all_pointclouds(float width, float height, glfw_state& app_state, multiFrame* m_frame)
{
    // OpenGL commands that prep screen for the pointcloud
    glPopMatrix();
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    
    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
    glClear(GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    gluPerspective(60, width / height, 0.01f, 10.0f);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);
    
    glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
    glRotated(app_state.pitch, 1, 0, 0);
    glRotated(app_state.yaw, 0, 1, 0);
    glTranslatef(0, 0, -0.5f);
    
    // exclude thread to work on frames
    std::lock_guard<std::mutex> guard(frames_mutex);
    
    for (int dNum = 0; dNum < m_frame->nframes(); dNum++) {
        rs2::video_frame* mf_color = m_frame->pull_color_frame(dNum);
        rs2::points* mf_points = m_frame->pull_frame(dNum);
        if (mf_color != NULL && mf_points != NULL) {
            std::cout << "pull: n " << dNum << ", c " << mf_color << ", p " << mf_points << '\n';
            // Upload the color frame to OpenGL
            app_state.tex.upload(*mf_color);
            drawit(width, height, app_state, *mf_points);
        }
        else
            std::cout << "pull: n " << dNum << ", c " << mf_color << ", p " << mf_points << '\n';
    }

    // OpenGL cleanup
    glEnd();
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
    glPushMatrix();
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
        
        while (capturingactive) // Application still alive?
        {
            // Wait for the next set of frames from the camera
            auto frames = pipe.wait_for_frames();
            auto depth = frames.get_depth_frame();
            auto color = frames.get_color_frame();

            // exclude main to use frames at this time
            std::lock_guard<std::mutex> guard(frames_mutex);
            
            // Generate the pointcloud and texture mappings
            points = pc.calculate(depth);
            
            // Tell pointcloud object to map to this color frame
            pc.map_to(color);
            
            m_frame->push_color_frame(&color, camNum);
            m_frame->push_frame(&points, camNum);
        }
        std::cout << "stopping camera ser no: " << serial_number << '\n';
        pipe.stop();
    }
    ).detach();
}


int main(int argc, char * argv[]) try
{
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Multicamera Capturing");
    // Construct an object to manage view state
    glfw_state app_state;
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);
    
    rs2::context ctx;    // Create librealsense context for managing devices
    
    auto devs = ctx.query_devices();

    multiFrame multiframe = *new multiFrame(devs.size());
    
    // Start capturing for all connected RealSense devices
    capturingactive = true;

    for (int dNum = 0; dNum < multiframe.nframes(); dNum++) {
        std::string serial_number(devs[dNum].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        int ret = multiframe.register_camera(serial_number);
        if (ret >= 0)
             capture_start(serial_number, &multiframe, dNum);
    }

    while (app) {
        // Draw the pointcloud(s)
        draw_all_pointclouds(app.width(), app.height(), app_state, &multiframe);
    }
    capturingactive = false;
    
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
