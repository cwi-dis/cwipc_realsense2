# cwipc_realsense2

This project has software for capturing point clouds using RealSense cameras. The software turns RealSense depth- and colorframes into a cwipc pointcloud.

It is usually built or installed as part of the _cwipc_ suite, <https://github.com/cwi-dis/cwipc>.

When installed it enables the use of RealSense D4xx cameras to tools like `cwipc_view`, `cwipc_grab`, `cwipc_register` and others. 

This module requires the [Intel Librealsense](https://github.com/IntelRealSense/librealsense) SDK. On Windows the needed files are included in the installer. On Linux you can install through `apt`, on MacOS through `brew`.

## Prepare for use

The realsense capturer needs a `cameraconfig.json` file that specifies the serial numbers of the cameras and their position and orientation.

If you have a single camera in landscape mode looking forward, approximately 1m from the subject at 1m height you can automatically create a config file with

```
cwipc_register --tabletop
```

In the unlikely event that `cwipc_register` does not recognize the fact that you have a Realsense camera you can supply the `--realsense` option.

If you have more cameras, or a single camera that is oriented differently (for example in portrait mode, so it is easier to capture a full human body) you should check the documentation on `cwipc_register`.

Inspect the resulting pointcloud view with

```
cwipc_view
```

## cameraconfig.json

You can edit `cameraconfig.json` and modify various settings, such as camera parameters like white balance, various processing options that govern how RGB and Depth images are converted to pointclouds, and bounding box parameters for the pointcloud.

After editing parameters you re-run `cwipc_view` to see the effect of your changes.