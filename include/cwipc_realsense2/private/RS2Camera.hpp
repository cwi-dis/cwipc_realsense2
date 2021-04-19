#ifndef cwipc_realsense_RS2Camera_hpp
#define cwipc_realsense_RS2Camera_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

#include <librealsense2/rs.hpp>

#include "cwipc_realsense2/private/RS2Config.hpp"

class RS2Camera {
private:
	RS2Camera(const RS2Camera&);	// Disable copy constructor
	RS2Camera& operator=(const RS2Camera&);	// Disable assignment
protected:
	RS2Camera(int _camera_index, rs2::context& ctx, RS2CaptureConfig& configuration, RS2CameraData& _camData);
public:
	RS2Camera(rs2::context& ctx, RS2CaptureConfig& configuration, int _camera_index, RS2CameraData& _camData, std::string _usb="0");
	virtual ~RS2Camera();

	void start();
	virtual void start_capturer();
	void stop();
	bool capture_frameset();
	void create_pc_from_frames();
	void wait_for_pc();
    void save_auxdata(cwipc *pc, bool rgb, bool depth);
	uint64_t get_capture_timestamp();
    cwipc_pcl_pointcloud get_current_pointcloud() { return current_pointcloud; }
public:
	// This is public because RS2Capture needs it when dumping the color images
	rs2::frameset current_frameset;
	float pointSize;
public:
	// These are public because pcl_align wants to access them
	double minx;
	double minz;
	double maxz;
	int camera_index;
	std::string serial;

protected:
	bool stopped;
	std::thread *processing_thread;
	void _computePointSize(rs2::pipeline_profile profile);
	void _processing_thread_main();
	virtual void _start_capture_thread();
	virtual void _capture_thread_main();
	rs2::frame_queue captured_frame_queue;
	void transformPoint(cwipc_pcl_point& out, const rs2::vertex& in);
    void _erode_depth(rs2::depth_frame, int x_delta, int y_delta);
private:
	RS2CameraData& camData;
	RS2CameraConfig& camSettings;
	bool high_speed_connection;
    cwipc_pcl_pointcloud current_pointcloud;

	int camera_width;
	int camera_height;
	int camera_fps;
	bool do_greenscreen_removal;
	bool do_height_filtering;
	double height_min;
	double height_max;

	std::thread *grabber_thread;
	rs2::frame_queue processing_frame_queue;
	std::mutex processing_mutex;
	std::condition_variable processing_done_cv;
	bool processing_done;

	rs2::pipeline pipe;
	bool pipe_started;
	// for an explanation of filtering see librealsense/doc/post-processing-filters.md and code in librealsense/src/proc
	rs2::align aligner;					// Align depth and color data
	rs2::decimation_filter dec_filter;                        // Decimation - reduces depth frame density
	rs2::threshold_filter threshold_filter;					  // Thresholding: minimum and maximum distance
	rs2::disparity_transform depth_to_disparity = rs2::disparity_transform(true);
	rs2::spatial_filter spat_filter;                          // Spatial    - edge-preserving spatial smoothing
	rs2::temporal_filter temp_filter;                         // Temporal   - reduces temporal noise
	rs2::disparity_transform disparity_to_depth = rs2::disparity_transform(false);
	rs2::pointcloud pointcloud;		// The pointcloud constructor

	void _init_filters();

};
#endif // cwipc_realsense_RS2Camera_hpp
