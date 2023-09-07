//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//

// Define to get (a little) debug prints
#undef CWIPC_DEBUG
#undef CWIPC_DEBUG_THREAD

// This is the dll source, so define external symbols as dllexport on windows.

#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif

#include "cwipc_realsense2/private/RS2Config.hpp"
#include "cwipc_realsense2/private/RS2OfflineCamera.hpp"

RS2OfflineCamera::RS2OfflineCamera(rs2::context& ctx, RS2CaptureConfig& configuration, int _camera_index, RS2CameraConfig& _camData, cwipc_rs2offline_settings& settings) :
  RS2Camera(_camera_index, ctx, configuration, _camData),
  depth_width(settings.depth.width),
  depth_height(settings.depth.height),
  depth_bpp(settings.depth.bpp),
  depth_fps(settings.depth.fps),
  depth_format(rs2_format(settings.depth.format)),
  color_width(settings.color.width),
  color_height(settings.color.height),
  color_bpp(settings.color.bpp),
  color_fps(settings.color.fps),
  color_format(rs2_format(settings.color.format)),
  depth_to_color_extrinsics({ { 1,0,0,0,1,0,0,0,1 },{ 0,0,0 } }),
  dev(),
  depth_sensor(dev.add_sensor("Depth")),
  color_sensor(dev.add_sensor("Color")),
  feedFrameNum(0)
{
    // Get transformation matrix between color and depth in librealsense2 format
    if (_camData.intrinsicTrafo == nullptr) {
        cwipc_rs2_log_warning("cwipc_rs2offline: camera has no intrinsicTrafo");
    } else {
        auto matrix = _camData.intrinsicTrafo->matrix();
        depth_to_color_extrinsics.rotation[0] = matrix(0, 0);
        depth_to_color_extrinsics.rotation[1] = matrix(0, 1);
        depth_to_color_extrinsics.rotation[2] = matrix(0, 2);
        depth_to_color_extrinsics.rotation[3] = matrix(1, 0);
        depth_to_color_extrinsics.rotation[4] = matrix(1, 1);
        depth_to_color_extrinsics.rotation[5] = matrix(1, 2);
        depth_to_color_extrinsics.rotation[6] = matrix(2, 0);
        depth_to_color_extrinsics.rotation[7] = matrix(2, 1);
        depth_to_color_extrinsics.rotation[8] = matrix(2, 2);
        depth_to_color_extrinsics.translation[0] = matrix(0, 3);
        depth_to_color_extrinsics.translation[1] = matrix(1, 3);
        depth_to_color_extrinsics.translation[2] = matrix(2, 3);
    }

    dev = rs2::software_device();

    // Create depth stream
    rs2_intrinsics depth_intrinsics = {
        depth_width, depth_height,
        (float)depth_width / 2, (float)depth_height / 2,
        (float)depth_width , (float)depth_height ,
        RS2_DISTORTION_BROWN_CONRADY,
        { 0,0,0,0,0 }
    };

    depth_stream = depth_sensor.add_video_stream({
        RS2_STREAM_DEPTH,
        0,
        0,
        depth_width, depth_height,
        depth_fps, depth_bpp,
        depth_format,
        depth_intrinsics
    });

    depth_sensor.add_read_only_option(RS2_OPTION_DEPTH_UNITS, 0.0001f);
    depth_sensor.add_read_only_option(RS2_OPTION_STEREO_BASELINE, 0.0001f);

#ifdef xxxjack
    auto ds = depth_sensor.as<rs2::depth_sensor>();
    auto dscale = ds.get_depth_scale();
    std::cerr << "dpeth scale=" << dscale << std::endl;
#endif

    // Create color stream
    rs2_intrinsics color_intrinsics = {
        color_width, color_height,
        (float)color_width / 2, (float)color_height / 2,
        (float)color_width, (float)color_height,
        RS2_DISTORTION_BROWN_CONRADY,
        { 0,0,0,0,0 }
    };

    color_stream = color_sensor.add_video_stream({
        RS2_STREAM_COLOR,
        0,
        1,
        color_width, color_height,
        color_fps, color_bpp,
        color_format,
        color_intrinsics
    });

    // Tie the two streams together
    dev.create_matcher(RS2_MATCHER_DEFAULT);
    depth_sensor.open(depth_stream);
    color_sensor.open(color_stream);
    depth_sensor.start(sync);
    color_sensor.start(sync);
    depth_stream.register_extrinsics_to(color_stream, depth_to_color_extrinsics);
}

RS2OfflineCamera::~RS2OfflineCamera() {}

void RS2OfflineCamera::_start_capture_thread() {
    // No capture thread needed for offline processing
}

void RS2OfflineCamera::_capture_thread_main() {
    assert(0);
}

bool RS2OfflineCamera::feed_image_data(int frameNum, void *colorBuffer, size_t colorSize,  void *depthBuffer, size_t depthSize) {
    // frameNum is ignored, because sometimes we have to feed a frame multiple times

    bool frameset_produced = false;
    while (!frameset_produced) {
        feedFrameNum++;

        depth_sensor.on_video_frame({
            depthBuffer,
            [](void *) {},
            depth_width * depth_bpp,
            depth_bpp,
            (rs2_time_t)(feedFrameNum * 1000.0 / depth_fps),
            RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK,
            feedFrameNum,
            depth_stream
        });

        color_sensor.on_video_frame({
            colorBuffer,
            [](void *) {},
            color_width*color_bpp,
            color_bpp,
            (rs2_time_t)(feedFrameNum * 1000.0 / color_fps),
            RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK,
            feedFrameNum,
            color_stream
        });

        // Wait to find if there is a next set of frames from the camera
        rs2::frameset fset = sync.wait_for_frames();
        auto depth = fset.first_or_default(RS2_STREAM_DEPTH);
        auto color = fset.first_or_default(RS2_STREAM_COLOR);

        if (depth && color) {
            captured_frame_queue.enqueue(fset);
            frameset_produced = true;
        }
    }

#ifdef CWIPC_DEBUG_THREAD
    std::cerr << "RS2OfflineCamera: fed camera " << serial << " framenum " << frameNum << " as " << feedFrameNum << std::endl;
#endif

  return true;
}
