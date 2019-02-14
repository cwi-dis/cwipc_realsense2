//
//  window_util.cpp
//
//  Created by Fons Kuijk on 14-02-19.
//

#define GLFW_INCLUDE_GLU
#include "GLFW/glfw3.h"
#include <iostream>
#include <functional>

class window_util
{
public:
	std::function<void(bool)>           on_left_mouse = [](bool) {};
	std::function<void(double, double)> on_mouse_scroll = [](double, double) {};
	std::function<void(double, double)> on_mouse_move = [](double, double) {};
	std::function<void(int)>            on_key_release = [](int) {};

	window_util(int width, int height, const char* title) : _width(width), _height(height)
	{
		glfwInit();
		win = glfwCreateWindow(width, height, title, nullptr, nullptr);
		if (!win)
			throw std::runtime_error("Could not open OpenGL window, please check your graphic drivers or use the textual SDK tools");
		glfwMakeContextCurrent(win);
		glfwSetWindowUserPointer(win, this);
		glfwSetMouseButtonCallback(win, [](GLFWwindow * win, int button, int action, int mods) {
			auto s = (window_util*)glfwGetWindowUserPointer(win);
			if (button == 0) s->on_left_mouse(action == GLFW_PRESS);
		});

		glfwSetScrollCallback(win, [](GLFWwindow * win, double xoffset, double yoffset) {
			auto s = (window_util*)glfwGetWindowUserPointer(win);
			s->on_mouse_scroll(xoffset, yoffset);
		});

		glfwSetCursorPosCallback(win, [](GLFWwindow * win, double x, double y) {
			auto s = (window_util*)glfwGetWindowUserPointer(win);
			s->on_mouse_move(x, y);
		});

		glfwSetKeyCallback(win, [](GLFWwindow * win, int key, int scancode, int action, int mods) {
			auto s = (window_util*)glfwGetWindowUserPointer(win);
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

	~window_util()
	{
		glfwDestroyWindow(win);
		glfwTerminate();
	}

	operator GLFWwindow*() { return win; }

private:
	GLFWwindow* win;
	int _width, _height;
};
