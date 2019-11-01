#ifndef cwipc_realsense_MFCamera_hpp
#define cwipc_realsense_MFCamera_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

#include <librealsense2/rs.hpp>

#include "defs.h"

class MFCamera {
private:
	MFCamera(const MFCamera&);	// Disable copy constructor
	MFCamera& operator=(const MFCamera&);	// Disable assignment
protected:
	MFCamera(int _camera_index, rs2::context& ctx, MFCaptureConfig& configuration, MFCameraData& _camData);
public:
	MFCamera(rs2::context& ctx, MFCaptureConfig& configuration, int _camera_index, MFCameraData& _camData, std::string _usb="0");
	virtual ~MFCamera();

	void start();
	virtual void start_capturer();
	void stop();
	bool capture_frameset();
	void create_pc_from_frames();
	void wait_for_pc();
	void dump_color_frame(const std::string& filename);
	uint64_t get_capture_timestamp();
public:
	// This is public because MFCapture needs it when dumping the color images
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
private:
	MFCameraData& camData;
	MFCameraSettings& camSettings;
	bool high_speed_connection;

	int camera_width;
	int camera_height;
	int camera_fps;
	bool do_depth_filtering;
	bool do_background_removal;
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
#endif // cwipc_realsense_MFCamera_hpp
