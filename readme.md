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
- You need cwipc_util
	- for example from <https://github.com/cwi-dis/cwipc_util> 
	- ... or <https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_util>
	- Use cmake and Visual Studio to build according to instructions there
	- ... or use a prebuilt installer if available.
	- Anyway, remember the installation directory. Suggested is `.../DIR/installed` where `DIR` is the directory where you have cloned all the repos.
	- And remember to add that directory to your system environment variable `PATH`.
- Create `build` subdirectory, run *cmake*, point it to your source and build directory, *Configure*.
	- A number of errors are expected and harmless:
		- *DAVIDSDK* not found
		- *DSSDK* not found
		- *ENSENSO* not found
		- *OPENNI* not found
		- *RSSDK* not found
	- Set `CMAKE_INSTALL_PREFIX` to the installation directory (the `.../DIR/installed` from above).
	- Configure again. There should be no more errors.
- Build the resultant Visual Studio solution.
- The outputs are going to end up in the `build` subdirectory.
	- More exact locations to be provided...
	- TBD: copy the outputs to a known location (for subsequent installing)

## MacOS

- You need XCode.
- Install Homebrew, from <https://brew.sh>
- Install a few dependencies needed by our software and some of the third party libraries:

  ```
  brew install cmake
  brew install homebrew/core/glfw3
  ```
  
- Then install the Intel RealSense SDK.
	- Look at <https://github.com/IntelRealSense/librealsense> and follow install instructions there for Mac.
	- I had to manually add the *libusb* search path the the XCode project (*cmake* did something wrong)
	- **This may fail on OSX 10.14...**, alternatively build with makefiles:
		- `mkdir build-makefiles`
		- `cd build-makefiles`
		- `LIBRARY_PATH=/usr/local/lib cmake .. -DBUILD_EXAMPLES=true -DBUILD_WITH_OPENMP=false -DHWM_OVER_XU=false -G "Unix Makefiles"`
		- `LIBRARY_PATH=/usr/local/lib make`
		- `make install`
		- Manually edit `/usr/local/lib/pkgconfig/realsense2.pc` and fix `libdir`.
- Install the PCL, easiest using brew: 

  ```
  brew install pcl
  ```
  
  - This may not work if your brew already uses PCL 1.9.x. Remove the too-new pcl, manually edit the formula and try to re-install again:

	  ```
	  brew uninstall pcl
	  cd /usr/local/Homebrew/Library/Taps/homebrew/homebrew-core/Formula
	  git log pcl.rb
	  # Note the commit of the last 1.8.1 formula, use that in the next line
	  git checkout baea3606fce5d96720f631f37d62662ea73d7798 -- pcl.rb
	  cd
	  brew install pcl
	  ```
  
- Now build our software. In this directory, *VRTogether-capture*, create `build`, go there.
- Run `cmake ..`.
	- That invocation creates Makefiles. To create an *Xcode* project use `cmake .. -G Xcode`.
	- dynamic library is in _build/src/libcwipc_realsense2.dylib_, test programs are in _build/renderer/pcl\_renderer_ and _pcl\_align_.
	- TBD: copy the outputs to a known location (for subsequent installing)

## Linux

- You need Ubuntu 18.04 (16.04 linuxes may work too)
- Install the Intel RealSense SDK following instructions for Linux from <https://github.com/IntelRealSense/librealsense>.
- Install _cmake_ with `sudo apt-get install cmake`.
- Install PCL (in the standard repos for 18.04), libusb:

  ```
  sudo apt-get install libpcl-dev libpcl-common1.8 libpcl-io1.8
  sudo apt-get install libusb-1.0 libusb-dev
  sudo apt-get install libglfw3 libglfw3-dev
  ```
- In this directory, *VRTogether-capture*, create `build`, go there.
- Run `cmake ..`.
	- More exact locations to be provided...
	- TBD: copy the outputs to a known location (for subsequent installing)


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
</file>
```
