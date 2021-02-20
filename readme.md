# cwipc_realsense2

This project has software for capturing point clouds using RealSense cameras. The software turns RealSense depth- and colorframes into a cwipc pointcloud.

It requires the cwipc_util library.

## Installing

For use within VRtogether you can get pre-built zipfiles (or tgzfiles for Mac/Linux) from <https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_realsense2/releases>. Download the most recent release with a normal v_X_._Y_._Z_ name. You will also need the accompanying _cwipc\_util_ installer from 
<https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_util/releases>.

[![pipeline status](https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_realsense2/badges/master/pipeline.svg)](https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_realsense2/commits/master)

### Windows

- Install cwipc_util (and its dependencies PCL and others).

- Install _librealsense2_ from <https://github.com/IntelRealSense/librealsense/releases>. You need the librealsense version that matches cwipc_realsense, currently 2.41.
- Extract the `cwipc_util_win1064_vX.Y.zip` file into the same place you extracted cwipc_util. Check that the DLLs are right next to each other in `installed/bin`
- Add the `c:\vrtogether\installed\bin` folder to the `%PATH%` system environment variable.

### OSX

- Install _brew_, and then

  ```
  brew install pcl
  brew install homebrew/core/glfw3
  brew install librealsense
  ```

- Extract both gzipped tar files (for _cwipc\_util_ and _cwipc\_realsense2_) in the `/usr/local` directory:

  ```
  cd /usr/local
  [sudo] tar xfv .../cwipc_util_osx1012_vX.Y.tgz
  [sudo] tar xfv .../cwipc_realsense2_osx1012_vX.Y.tgz
  ```
  
### Ubuntu 20.04

- Install _PCL_ with `apt-get install libpcl-dev`.
- Install _librealsense_ following instructions at <https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md> for Ubuntu 20.04.
- Extract both gzipped tar files (for _cwipc\_util_ and _cwipc\_realsense2_) in the `/usr/local` directory:

  ```
  cd /usr/local
  [sudo] tar xfv .../cwipc_util_osx1012_vX.Y.tgz
  [sudo] tar xfv .../cwipc_realsense2_osx1012_vX.Y.tgz
  ```



## Building from source

The directory `cwipc_realsense2` that is the root of this repository should be put on your system where you cloned all other repos. The CMake setup should be able to find all dependencies needed.

It is assumed that if you build cwipc_realsense2 from source you also build cwipc_util from source. So the instructions here assume that you have already followed the cwipc_util instructions.

## Build instructions (all platforms)

- You need everything needed by the cwipc_util build instructions.
- You need Intel Realsense SDK from <https://github.com/IntelRealSense/librealsense>. On windows: manually add the location of its 64 bit dll (e.g. `C:\Program Files (x86)\Intel RealSense SDK 2.0\bin\x64`) to your `PATH`
- After you have built cwipc_util you follow the same sequence of commands for cwipc_realsense2.


## Expected input: *cameraconfig.xml*

**Note** this section is outdated, not all of the information here is correct.

The configuration file *cameraconfig.xml* specifies system parameters type of processing, parameters for depth filtering and camera data.

- System parameters are width, heigth and framerate for usb2 and usb3 connections and the number of output buffers (an int).
- The processing that can be switched on/of is depthfiltering (for denoising depth values), backgroundremoval, greenscreenremoval and tiling (not available yet) this goes with parameter `tileresolution`. The parameter `cloudresolution` if not 0 can be used to reduce cloudsize.
- Parameters for depth filtering.
- Information for each camera is the serial number, the position of the background plane and the transformation for alignment.

Below is an example config file.
```
<?xml version="1.0" ?>
<file>
    <CameraConfig>
        <system usb2width="640" usb2height="480" usb2fps="15" usb3width="1280" usb3height="720" usb3fps="30" />
        <!-- 'cloudresolution' and 'tileresolution' are specified in meters -->
        <postprocessing depthfiltering="1" backgroundremoval="1" greenscreenremoval="1" cloudresolution="0" tiling="0" tileresolution="0.01">
            <!-- For information on depth filtering parameters see librealsense/doc/post-processing-filters.md -->
            <!--	decimation_value is an int between 2 and 8 -->
            <!--	spatial_iterations is an int between 1 and 5 -->
            <!--	spatial_alpha is is a float between 0.25 and 1.0 -->
            <!--	spatial_delta is an int between 1 and 50 -->
            <!--	spatial_filling is an int between 0 and 6 -->
            <!--	temporal_alpha is is a float between 0 and 1 -->
            <!--	temporal_delta is is an int between 1 and 100 -->
            <!--	temporal_percistency is a float between 0 and 8 -->
            <depthfilterparameters decimation_value="2" spatial_iterations="4" spatial_alpha="0.25" spatial_delta="30" spatial_filling="0" temporal_alpha="0.4" temporal_delta="20" temporal_percistency="3" />
        </postprocessing>
        <!-- backgroundx, backgroundy and backgroudz if not 0 position the camera's background plane -->
        <camera serial="802212060048" backgroundx="0.733779" backgroundy="0" backgroundz="2.18281">
            <trafo>
                <values 
                v00="0.278341" v01="0.0725892" v02="-0.957736" v03="-0.794437"
                v10="0.0177684" v11="0.99658" v12="0.0806973" v13="0.0773968"
                v20="0.960318" v21="-0.0394787" v22="0.276099" v23="-0.458746"
                v30="0" v31="0" v32="0" v33="1" />
            </trafo>
        </camera>
        <camera serial="746112061997" backgroundx="0.0" backgroundy="0.0" backgroundz="0.0">
            <trafo>
                <values 
                v00="1" v01="0" v02="0" v03="0"
                v10="0" v11="1" v12="0" v13="0"
                v20="0" v21="0" v22="1" v23="0"
                v30="0" v31="0" v32="0" v33="1" />
            </trafo>
        </camera>
        <camera serial="746112060605" backgroundx="0.0" backgroundy="0.0" backgroundz="0.0">
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

## Special feature:

By setting the environment variable `CWI_CAPTURE_FEATURE` to *dumpvideoframes* the capturer will write out the video frames as .pgn files named "videoframe\_*timestamp*\_*cameraindex*.png"

## Calibrating alignment

**Note** This section is outdated and incorrect.

Aligning multiple cameras is a bit of a black art. Here is an attempt at a description, we assume 4 cameras in the rest of this text, but other numbers of cameras should work just fine. There is a program `cwipc_calibrate` included in `cwipc_util` that helps with the procedure.

- Mount your cameras in four corners, looking in. For capturing people mounting them at a 90 degree angle works best. Spacing the cameras about 3 meters apart seems to work. But again: other configurations should also work.
- Add sync cables to the cameras.
- Build a calibration device. _Cross V3_ is best, but there are a number of alternatives:
	- _Cross V2_ does not have LEDs, and may be easier to build.
	- _A4 paper_ is even easier, but does not work very well when viewed from the side. In `cwipc_util/doc/target-a4.pdf` there is a file you can print double-sided that will help you.
	
  You can run `cwipc_calibrate --list` to see all supported calibration devices.
- Create a working directory and go there:

  ```
  mkdir calibration
  cd calibration
  ```
- Place the cross at the origin, upright, and with the "front arm" pointing in the "natural viewing direction".
- Run the following command to grab the first set of 4 pointclouds, still completely unaligned:
  
  ```
  cwipc_calibrate --nocoarse --nofine --depth 0.5,3
  ```
  The numbers `0.5` and `3` are the minimum and maximum
  distance from the camera at which points will be captured _for future captures_, in meters. 
  You will be shown a preview of each camera capture, visually inspect these previews to ascertain you can select the points of the cross.
- Optionally, if there is too much background, do another capture with the previously given depths (which should get rid of background, making it easier to select the points):

  ```
  cwipc_calibrate --nocoarse --nofine --reuse
  ```
- Now you can do the coarse calibration. In this procedure, you select the colored points on the reference pointcloud, and then on each per-camera pointcloud you select the same points (in the same order). This step will create a per-camera rotation and translation matrix that will somewhat align each camera with the wanted coordinate system. You will be shown 10 pointclouds:
	- Reference pointcloud. Here you select the colored balls or LEDs or corners of the A4 sheet in the order you want.
	- Captured pointcloud of camera 1. Here you select the balls/LEDs/corners in the same order as for the reference pointcloud.
	- Translated/rotated pointcloud from camera 1 to world coordinates. Inspect that it is as you wanted.
	- The previous two steps are repeated for every camera.
	- As the last step you are shown the fused pointcloud of all cameras.
  
  During the coarse calibration you can also set the minimum and maximum height of your future fused captured pointclouds (again in meters, with 0 being ground level). This allows you to remove floor and ceiling from your captures.
  
  Here is the command to run the coarse calibration step:
  
  ```
  cwipc_calibrate --reuse --nograb cwipc_calibrate_captured.ply --noinspect --nofine --crossv3 --height 0.05,2.2
  ```
- You can now use the viewer to look at live captures:

  ```
  cwipc_view
  ```
  You can use commands _01248_ to select a certain camera (tile), space to pause/resume, _w_ to save a pointcloud.
- Now you can do fine calibration. Remove the cross and put a human being there. Use `cwipc_view` to inspect what the cameras see. Once you have a good picture use `w` to save a pointcloud, the file will  be named something like `pointcloud_123456789.ply`. You may want to inspect it after capturing with something like `meshlab`.
- Do the fine calibration on the captured human. This should snap the 4 pointclouds together by doing very small rotations and translations. This is currently an interactive process where you select an algorithm and some parameters. After each run you get shown the results. Continue until you get something you like.

	You should _not_ run this step in `bash` but use the Windows command prompt, at the moment, because in `bash` the interactivity will not work.
	
	```
	cwipc_calibrate --reuse --nograb pointcloud_123456789.ply --noinspect --nocoarse
	```

- You can always rerun the fine calibration. If you ever want to rerun the coarse calibration you should pass the `--clean` argument to the first step (the capturing) so the old calibration is ignored.
- All information from the calibration process is contained in the `cameraconfig.xml` file. You can now copy this from your temporary calibration working directory to where you want to use it (and keep the calibration working directory with all its plyfiles for future reference).