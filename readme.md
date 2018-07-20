# VRTogether-capture
Software for capturing point clouds using RealSense cameras

## Overview

This software serves to turn RealSense frames (of one or more RealSense cameras) into a PCL pointcloud. It goes with an app to visualize the 3D pointcloud .

## Expected Output

This project outputs a dll named `multiFrame.dll` that gives access to the PCL formatted pointcloud. The function to get a pointcloud together with a timestamp can be found in the .dll is `getPointCloudFunction)(long *, void **)`

Otherwise, for visual inspection of the result, the project can generate an application `pcl_renderer` that opens a window with a pointcloud. Using your mouse, you are able to interact with the pointcloud, rotating, zooming, and panning.

## Installation

The directory `VRTogether-capture` that is maintained by this repository can be put anywhere on your system.
The CMake setup should be able to find the dependencies needed.``


