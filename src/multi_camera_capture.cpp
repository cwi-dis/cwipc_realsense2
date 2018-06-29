// License: Apache 2.0. See LICENSE file in root directory.

#include "multiFrame.hpp"
#include "example.hpp"          // Include short list of convenience functions for rendering

int frameNum;

// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);

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
        
    for (int dNum = 0; dNum < m_frame->n_cameras(); dNum++) {
        rs2::video_frame* mf_color = m_frame->pull_color_frame(dNum);
        rs2::points* mf_points = m_frame->pull_frame(dNum);
        if (mf_color != NULL && mf_points != NULL) {
            // Upload the color frame to OpenGL
            app_state.tex.upload(*mf_color);
            drawit(width, height, app_state, *mf_points);
        }
    }

    // OpenGL cleanup
    glEnd();
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
    glPushMatrix();
}

void cloud2file(PointCloudT pntcld)
{
    int size = pntcld.size();
    if (size <= 0) return;
    
    std::ofstream myfile(("pcl_frame" + std::to_string(frameNum++) + ".ply").c_str());
    myfile << "ply\n" << "format ascii 1.0\nelement vertex " << size << "\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
    
    std::ostringstream oss;
    for (int i = 0; i < size; i++)
    {
        oss << (std::to_string(pntcld[i].x) + " " +
                std::to_string(pntcld[i].y) + " " +
                std::to_string(pntcld[i].z) + " " +
                std::to_string(pntcld[i].r) + " " +
                std::to_string(pntcld[i].g) + " " +
                std::to_string(pntcld[i].b) + "\n").c_str();
    }
    myfile << oss.str();
    myfile.close();
}

int main(int argc, char * argv[]) try
{
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Multicamera Capturing");
    // Construct an object to manage view state
    glfw_state app_state;
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);
    
    multiFrame multiframe;

    frameNum = 0;
    while (app) {
        // Draw the live frames
        draw_all_pointclouds(app.width(), app.height(), app_state, &multiframe);
        
        // As test write a ply file of the pointcloud
        PointCloudT pntCloud = multiframe.getPointCloud();
        cloud2file(pntCloud);
    }
    
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
