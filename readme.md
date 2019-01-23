# VRTogether-capture
A CMake project generating software for capturing point clouds using RealSense cameras.

[TOC]

## Overview

This CMake project produces software to turn RealSense frames (of one or more RealSense cameras) into a PCL pointcloud. It goes with two apps to visualize the 3D pointcloud captured.
If the system has more depth cameras connected, a configuration file named *cameraconfig.xml* is expected (for details see below).

## Expected Output: DLL and Applications

#### 1) The dll file: *multiFrame.dll*

This project produces a dll named `multiFrame.dll` that gives access to the PCL formatted pointcloud. The function to get a pointcloud together with a timestamp can be found in the .dll is `getPointCloud(long * timestamp, void ** pointcloud)`

For those interested, a pointcloud in PCL format can be obtained from the `void** pointcloud` by casting: `captured_pc = *reinterpret_cast<boost::shared_ptr<PointCloud<PointXYZRGB>>*>(pointcloud)`

#### 2) The applications: *pcl_renderer* and *pcl_align*

For visual inspection of the result, the project generates two applications *pcl_renderer* and  *pcl_align* that both open a window showing the pointcloud.

*Pcl_renderer* serves as an example on how to make use of the dll and also is a means to test the capturing software. For this it accesses the capturing software using the public API only (using the function `getPointCloud(long * timestamp, void ** pointcloud)`). The pointcloud rendered by this application will automatically be centered with the view origin.
To examine the pointcloud the user can use the mouse: rightclick and move to rotate the pointcloud, and use the mouse wheel to zoom. The *esc* button will reset the position of the (fused) pointcloud.

*Pcl_align* serves another purpose. It can be used to produce a snapshot of the individual cameras and can be used to manually align the cameras by manipulating the individual transformation settings. When done, it can write out these settings to a file (*cameraconfiguration.xml*).
Action keys for alignment of camera clouds are:

- "a" to toggle between *life* and *alignment mode*;
- "1-9" to select the camera to align;
- "r" to start cloud rotate mode;
- "t" to start cloud translate mode;
- "esc" to reset the cloud transformation of the active camera";
- "s" to save the current configuration in *camaraconfig.xml* and snapshots of each camera in .ply files;
- "h" to print the help;
- "q" to quit;

## Building

### Windows

- You need Windows 10 64bit.
- You need Visual Studio 2017, community edition.
- You need CMake.
- You need Intel Realsense SDK.
- You need PCL 1.8.1, get from <https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.8.1>, AllInOne win64 installer.
- Create `build` subdirectory, run *cmake*, point it to your source and build directory, *Configure*.
	- A number of errors are expected and probably harmless:
		- *DAVIDSDK* not found
		- *DSSDK* not found
		- *ENSENSO* not found
		- *OPENNI* not found (but note that *OPENNI2* is needed)
		- *RSSDK* not found
- Build the resultant Visual Studio solution.
- The outputs are going to end up in the `build` subdirectory.
	- More exact locations to be provided...

## MacOS

- You need XCode.
- First install the Intel RealSense SDK.
	- Follow build and install instructions there for Mac. This will install *brew* and *cmake* and various other dependencies needed.
	- I had to manually add the *libusb* search path the the XCode project (*cmake* did something wrong)
	- **This will fail on OSX 10.14...**
- In this directory, *VRTogether-capture*, create `build`, go there.
- Run `cmake ..`.
- More to be provided...

## Linux

- You need Ubuntu 16.04 (other linuxes may work too)
- Install the Intel RealSense SDK following instructions for Linux from <https://github.com/IntelRealSense/librealsense>.
- Install _cmake_ with `sudo apt install cmake`.
- 

## Installation

The directory `VRTogether-capture` that is maintained by this repository can be put anywhere on your system.
The CMake setup should be able to find the dependencies needed.

## Expected input: *cameraconfig.xml*

The configuration file *cameraconfig.xml* holds information on cloud resolution (in meters), the number of output buffers (an int) and optional greenscreenremoval (0 or 1). Furthermore the file holds information for each camera: the serial number and the transformation for alignment. Below is an example config file.
```
<?xml version="1.0" ?>
<file>
    <CameraConfig resolution="0" ringbuffersize="1" greenscreenremoval="1">
        <camera serial="802212060048">
            <trafo>
                <values 
                v00="0.278341" v01="0.0725892" v02="-0.957736" v03="-0.794437"
                v10="0.0177684" v11="0.99658" v12="0.0806973" v13="0.0773968"
                v20="0.960318" v21="-0.0394787" v22="0.276099" v23="-0.458746"
                v30="0" v31="0" v32="0" v33="1" />
            </trafo>
        </camera>
        <camera serial="746112061997">
            <trafo>
                <values 
                v00="1" v01="0" v02="0" v03="0"
                v10="0" v11="1" v12="0" v13="0"
                v20="0" v21="0" v22="1" v23="0"
                v30="0" v31="0" v32="0" v33="1" />
            </trafo>
        </camera>
        <camera serial="746112060605">
            <trafo>
                <values 
                v00="0.731788" v01="-0.0250268" v02="0.681072" v03="0.670926"
                v10="-0.0533222" v11="0.99416" v12="0.0938244" v13="0.134867"
                v20="-0.679443" v21="-0.104976" v22="0.72618" v23="-0.0340216"
                v30="0" v31="0" v32="0" v33="1" />
            </trafo>
        </camera>
    </CameraConfig>
</file>```
