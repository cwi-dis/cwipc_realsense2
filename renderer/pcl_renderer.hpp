//
//  pcl_renderer.hpp
//
//  Created by Fons Kuijk on 12-12-18.
//

#ifndef pcl_renderer_hpp
#define pcl_renderer_hpp
#pragma once

#define GLFW_INCLUDE_GLU
#include "GLFW/glfw3.h"
#include "cwipc_realsense/defs.h"
#ifdef WIN32
#include <windows.h>
#else
extern "C" void getPointCloud(uint64_t *timestamp, void **pointcloud);
#endif
#include <librealsense2/rs.hpp>

typedef void(*GetPointCloudFunction)(uint64_t *, void **);

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
	cout << "Use \"q\" to quit\n";
}


// Handle the OpenGL setup needed to display all pointclouds
void draw_pointcloud(window& app, glfw_state& app_state, boost::shared_ptr<PointCloudT> point_cloud)
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

	for (auto pnt : point_cloud->points) {
		float col[] = { (float)pnt.r / 256.f, (float)pnt.g / 256.f, (float)pnt.b / 256.f };
		glColor3fv(col);
		float vert[] = { pnt.x, pnt.y, pnt.z };
		glVertex3fv(vert);
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
void register_glfw_callbacks(window& app, glfw_state& app_state)
{
	app.on_left_mouse = [&](bool pressed) {
		app_state.ml = pressed;
	};

	app.on_mouse_scroll = [&](double xoffset, double yoffset) {
		app_state.offset -= static_cast<float>(yoffset);
	};

	app.on_mouse_move = [&](double x, double y) {
		if (app_state.ml) {

			app_state.yaw += (x - app_state.last_x) / 10.0;
			app_state.pitch += (y - app_state.last_y) / 10.0;
			app_state.pitch = std::max(app_state.pitch, -85.0);
			app_state.pitch = std::min(app_state.pitch, +85.0);
		}
		app_state.last_x = x;
		app_state.last_y = y;
	};

	app.on_key_release = [&](int key) {
		if (key == 256) { // Escape is interpreted as a reset of the transformation

			app_state.yaw = app_state.pitch = 0;
			app_state.offset = 0.0;
		}
		else if (key == 81) {	// key = "q": Quit program
			exit(0);
		}
	};
}

#endif /* pcl_renderer_hpp */
