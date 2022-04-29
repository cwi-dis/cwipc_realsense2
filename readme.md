# cwipc_realsense2

This project has software for capturing point clouds using RealSense cameras. The software turns RealSense depth- and colorframes into a cwipc pointcloud.

It is usually built or installed as part of the _cwipc_ suite, <https://github.com/cwi-dis/cwipc>.

When installed it enables the `--realsense` option to tools like `cwipc_view`, `cwipc_grab`, `cwipc_calibrate` and others. This will use Intel Realsense cameras as a source of pointclouds.

> As of this writing the `--realsense` option is not implemented. Simply _not_ specifying a camera option will select the Realsense grabber.

This module requires the [Intel Librealsense](https://github.com/IntelRealSense/librealsense) SDK to be installed. On Windows install it using the binary installer and ensure the `bin\x64` directory is added to `%PATH%`. On Linux you can install through `apt`, on MacOS through `brew`.

## Prepare for use

The realsense capturer needs a `cameraconfig.xml` file that specifies the serial numbers of the cameras and their position and orientation.

If you have a single camera in landscape mode looking forward, approximately 1m from the subject at 1m height you can automatically create a config file with

```
cwipc_calibrate --auto
```

If you have more cameras, or a single camera that is oriented differently (for example in portrait mode, so it is easier to capture a full human body) you need to print a _target_ from `../cwipc_util/target-a4.pdf`. Easiest is to put the target on the floor, at the position that will become `(0, 0, 0)` in your captured pointclouds. Then run

```
cwipc_calibrate --target a4floor --nofine
```

> It is possible to hold the target in a different position, or use different targets, depending on the physical layout of your space. Use `cwipc_calibrate --list` to see the options.

You now inspect the frame captured to check that the target is fully visible in all cameras. If it is not you reorient and restart the calibration.

Next you select the 4 colored points on the virtual target, remembering the colors.

Then, for each of the cameras, you select the corners of the A4 paper with the same colors in the same order, and inspect the single-camera result.

Finally you inspect the merged pointcloud.

This gives you a course calibration in `cameraconfig.xml`.

Inspect the resulting pointcloud view with

```
cwipc_view
```

If you want to fine-tune calibration you can grab a pointcloud with the `w` key. You can then use 

```
cwipc_view --nograb file.ply --reuse --nocoarse
```

to attempt fine calibration but this is a bit of a black art.

## cameraconfig.xml

You can edit `cameraconfig.xml` and modify various settings, such as camera parameters like white balance, various processing options that govern how RGB and Depth images are converted to pointclouds, and bounding box parameters for the pointcloud.

After editing parameters you re-run `cwipc_view` to see the effect of your changes.