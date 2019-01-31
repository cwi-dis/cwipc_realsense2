//
//  pcl_align.hpp
//
//  Created by Fons Kuijk on 12-12-18.
//

#ifndef pcl_align_hpp
#define pcl_align_hpp
#pragma once

#include "cwipc_realsense/multiFrame.hpp"

#define GLFW_INCLUDE_GLU
#include "GLFW/glfw3.h"

#include <string>
#include <sstream>
#include <iostream>

class window
{
public:
	std::function<void(bool)>           on_left_mouse = [](bool) {};
	std::function<void(double, double)> on_mouse_scroll = [](double, double) {};
	std::function<void(double, double)> on_mouse_move = [](double, double) {};
	std::function<void(int)>            on_key_release = [](int) {};

	window(int width, int height, const char* title) : _width(width), _height(height)
	{
		glfwInit();
		win = glfwCreateWindow(width, height, title, nullptr, nullptr);
		if (!win)
			throw std::runtime_error("Could not open OpenGL window, please check your graphic drivers or use the textual SDK tools");
		glfwMakeContextCurrent(win);
		glfwSetWindowUserPointer(win, this);
		glfwSetMouseButtonCallback(win, [](GLFWwindow * win, int button, int action, int mods) {
			auto s = (window*)glfwGetWindowUserPointer(win);
			if (button == 0) s->on_left_mouse(action == GLFW_PRESS);
		});

		glfwSetScrollCallback(win, [](GLFWwindow * win, double xoffset, double yoffset) {
			auto s = (window*)glfwGetWindowUserPointer(win);
			s->on_mouse_scroll(xoffset, yoffset);
		});

		glfwSetCursorPosCallback(win, [](GLFWwindow * win, double x, double y) {
			auto s = (window*)glfwGetWindowUserPointer(win);
			s->on_mouse_move(x, y);
		});

		glfwSetKeyCallback(win, [](GLFWwindow * win, int key, int scancode, int action, int mods) {
			auto s = (window*)glfwGetWindowUserPointer(win);
			if (0 == action) // on key release
				s->on_key_release(key);
		});
	}

	float width() const { return float(_width); }
	float height() const { return float(_height); }

	operator bool()
	{
		glPopMatrix();
		glfwSwapBuffers(win);

		auto res = !glfwWindowShouldClose(win);

		glfwPollEvents();
		glfwGetFramebufferSize(win, &_width, &_height);

		// Clear the framebuffer
		glClear(GL_COLOR_BUFFER_BIT);
		glViewport(0, 0, _width, _height);

		// Draw the images
		glPushMatrix();
		glfwGetWindowSize(win, &_width, &_height);
		glOrtho(0, _width, _height, 0, -1, +1);

		return res;
	}

	~window()
	{
		glfwDestroyWindow(win);
		glfwTerminate();
	}

	operator GLFWwindow*() { return win; }

private:
	GLFWwindow* win;
	int _width, _height;
};

// Struct for managing rotation of pointcloud view
struct glfw_state {
	glfw_state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0), ml(false), offset(0.f) {}
	double yaw;
	double pitch;
	double last_x;
	double last_y;
	bool ml;
	float offset;
};

bool do_align = false;
bool rotation = true;
int aligncamera = 0;
Eigen::Vector4f mergedcenter;	// needed to automatically center the merged cloud
Eigen::Vector4f cloudcenter;		// needed to be able to rotate around the cloud's centre of mass

void printhelp() {
	cout << "\nThe cloud rendered by this application will automatically be centered with the view origin.\n";
	cout << "To examine the pointcloud use the mouse: leftclick and move to rotate, use the mouse wheel to zoom.\n";
	cout << "Use \"esc\" to reset the position of the (fused) cloud.\n";

	cout << "\naction keys for alignment of camera clouds:\n";
	cout << "\t\"a\" to toggle between \"life\" and \"alignment mode\"\n";
	cout << "\t\"1-9\" to select the camera to align\n";
	cout << "\t\"r\" to start cloud rotate mode\n";
	cout << "\t\"t\" to start cloud translate mode\n";
	cout << "\t\"esc\" to reset the cloud transformation of the active camera\n";
	cout << "\t\"s\" to save configuration and snapshots of each camera\n";
	cout << "\t\"h\" to print this help\n";
	cout << "\t\"q\" to quit\n";
}

void cloud2file(boost::shared_ptr<PointCloudT> pntcld, string filename)
{
	if (!pntcld) return;
	int size = pntcld->size();
	if (size <= 0) return;

	std::ofstream myfile(filename.c_str());
	myfile << "ply\n" << "format ascii 1.0\nelement vertex " << size << "\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";

	std::ostringstream oss;
	for (int i = 0; i < size; i++) {
		oss << (std::to_string((*pntcld)[i].x) + " " +
			std::to_string((*pntcld)[i].y) + " " +
			std::to_string((*pntcld)[i].z) + " " +
			std::to_string((*pntcld)[i].r) + " " +
			std::to_string((*pntcld)[i].g) + " " +
			std::to_string((*pntcld)[i].b) + "\n").c_str();
	}
	myfile << oss.str();
	myfile.close();
}

// Handle the OpenGL setup needed to display all pointclouds
void draw_pointcloud(window& app, glfw_state& app_state, multiFrame& multiframe)
{
	// OpenGL commands that prep screen
	glPopMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
	glClear(GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	gluPerspective(60, app.width() / app.height(), 0.01f, 10.0f);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	gluLookAt(0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

	glTranslatef(0.0, 0.0, app_state.offset*0.05f);
	glRotated(app_state.pitch, 1, 0, 0);
	glRotated(app_state.yaw, 0, 1, 0);
	glTranslatef(-mergedcenter.x(), -mergedcenter.y(), -mergedcenter.z());
	glPushMatrix();
	glPointSize(app.width() / 640);
	glEnable(GL_DEPTH_TEST);
	glBegin(GL_POINTS);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);

	// draw the pointcloud(s)
	if (do_align) {
		int cams = multiframe.getNumberOfCameras();
		for (int i = 0; i < cams; i++) {
			PointCloudT::Ptr pcptr(new PointCloudT);

			transformPointCloud(*(multiframe.getCameraCloud(i).get()), *pcptr, *multiframe.getCameraTransform(i));
			for (auto pnt : pcptr->points) {
				float col[3];
				if (i == aligncamera) {	// highlight the cloud of the selected camera
					col[0] = 0.2 + pnt.r / 320.0;
					col[1] = 0.2 + pnt.g / 320.0;
					col[2] = 0.2 + pnt.b / 320.0;
				}
				else {
					col[0] = pnt.r / 320.0;
					col[1] = pnt.g / 320.0;
					col[2] = pnt.b / 320.0;
				}
				glColor3fv(col);
				float vert[] = { pnt.x, pnt.y, pnt.z };
				glVertex3fv(vert);
			}
		}
	}
	else {
		for (auto pnt : multiframe.getPointCloud()->points) {
			float col[] = { (float)pnt.r / 256.f, (float)pnt.g / 256.f, (float)pnt.b / 256.f };
			glColor3fv(col);
			float vert[] = { pnt.x, pnt.y, pnt.z };
			glVertex3fv(vert);
		}
	}

	// OpenGL cleanup
	glEnd();
	glPopMatrix();
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
	glPushMatrix();
}

// Registers the state variable and callbacks to allow mouse control of the pointcloud
void register_glfw_callbacks(window& app, glfw_state& app_state, multiFrame& multiframe)
{
	app.on_left_mouse = [&](bool pressed) {
		app_state.ml = pressed;
	};

	app.on_mouse_scroll = [&](double xoffset, double yoffset) {
		if (do_align) {
			Eigen::Affine3d *transform = multiframe.getCameraTransform(aligncamera);
			if (rotation) {
				(*transform).rotate(Eigen::AngleAxisd(yoffset / 100.0, Eigen::Vector3d::UnitZ()));
			}
			else {
				(*transform).translate(Eigen::Vector3d(0.0, 0.0, -yoffset / 100.0));
			}
		}
		else {
			app_state.offset -= static_cast<float>(yoffset);
		}
	};

	app.on_mouse_move = [&](double x, double y) {
		if (app_state.ml) {
			if (do_align) {
				Eigen::Affine3d *transform = multiframe.getCameraTransform(aligncamera);
				double dx = (x - app_state.last_x) / (0.25 * app.width());
				double dy = -(y - app_state.last_y) / (0.25 * app.width());
				if (rotation) {
					(*transform).translate(Eigen::Vector3d(cloudcenter.x(), cloudcenter.y(), cloudcenter.z()));
					(*transform).rotate(Eigen::AngleAxisd(dx, Eigen::Vector3d::UnitY()));
					(*transform).rotate(Eigen::AngleAxisd(-dy, Eigen::Vector3d::UnitX()));
					(*transform).translate(-Eigen::Vector3d(cloudcenter.x(), cloudcenter.y(), cloudcenter.z()));
				}
				else {
					(*transform).translate(Eigen::Vector3d(dx, dy, 0.0));
				}
			}
			else {
				app_state.yaw += (x - app_state.last_x) / 10.0;
				app_state.pitch += (y - app_state.last_y) / 10.0;
				app_state.pitch = std::max(app_state.pitch, -85.0);
				app_state.pitch = std::min(app_state.pitch, +85.0);
			}
		}
		app_state.last_x = x;
		app_state.last_y = y;
	};

	app.on_key_release = [&](int key) {
		if (key == 256) { // Escape is interpreted as a reset of the transformation
			if (do_align) {
				Eigen::Affine3d *transform = multiframe.getCameraTransform(aligncamera);
				(*transform).setIdentity();
			}
			else {
				app_state.yaw = app_state.pitch = 0;
				app_state.offset = 0.0;
			}
		}
		else if (key == 65) {	// key = "a": start/stop Alignment
			if (do_align) {
				multiframe.capture_start();
				do_align = false;
			}
			else {
				multiframe.pauze_capture();
				do_align = true;
				pcl::compute3DCentroid((*multiframe.getCameraCloud(aligncamera)), cloudcenter);
			}
		}
		else if (key == 72) {	// key =\"h": print help
			printhelp();
		}
		else if (key == 81) {	// key = "q": Quit program
			multiframe.~multiFrame();
			exit(0);
		}
		else if (key == 82) {	// key = "r": Rotate
			rotation = true;
		}
		else if (key == 83) {	// key = "s": Save config and snapshots to file
			multiframe.config2file();
			int cams = multiframe.getNumberOfCameras();
			for (int i = 0; i < cams; i++)
				cloud2file(multiframe.getCameraCloud(i), multiframe.getCameraSerial(i) + ".ply");
		}
		else if (key == 84) {	// key = "t": Translate
			rotation = false;
		}
		else if (key >= 49 && key < multiframe.getNumberOfCameras() + 49) {	// key = "1-9": select a camera
			aligncamera = key - 49;
			pcl::compute3DCentroid((*multiframe.getCameraCloud(aligncamera)), cloudcenter);
		}
		else if (key == 73) {	// key =\"i": dump frames for icp processing
			int cams = multiframe.getNumberOfCameras();
			for (int i = 0; i < cams; i++) {
				PointCloudT::Ptr point_cloud_ptr(new PointCloudT);
				boost::shared_ptr<PointCloudT> aligned_cld(point_cloud_ptr);

				transformPointCloud(*(multiframe.getCameraCloud(i).get()), *aligned_cld, *multiframe.getCameraTransform(i));

				cloud2file(aligned_cld, "pcl_aligned_" + multiframe.getCameraSerial(i) + ".ply");
				cloud2file(multiframe.getCameraCloud(i), "pcl_original_" + multiframe.getCameraSerial(i) + ".ply");
			}
		}
	};
}

#endif /* pcl_align_hpp */
