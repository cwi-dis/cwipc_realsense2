# VRTogether-capture
Software for capturing point clouds using RealSense cameras

## Overview

This software serves to generate and visualize a 3D pointcloud using multiple RealSense cameras.

## Expected Output
The application should open a window with a pointcloud. Using your mouse, you should be able to interact with the pointcloud, rotating, zooming, and panning.

## Installation

The directory `VRTogether-capture` that is maintained by this repository should be put at the top level of the RealSense code tree, that is, it should be in the directory `librealsense` next to `examples`, `tools`, `src`, etc.

In the file `CMakeLists.txt` in that directory you have to add a line:

```
add_subdirectory(VRTogether-capture)
```
You can do that just before the line

```
# Check for unreferenced files
```
Then you can run the build instruction for the whole project. In the build folder you can zoom in to the VRTogether subtree and find a Capture-CWI project file to start Xcode or MS Visual Studio.

