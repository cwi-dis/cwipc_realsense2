import sys
import os
import cwipc
import cwipc.realsense2

from .pointcloud import Pointcloud
from .cameraconfig import CameraConfig

DEBUG=False

SKIP_FIRST_GRABS=10 # Skip this many grabs before using one. Needed for D435, it seems.

class LiveGrabber:
    def __init__(self):
        self.cameraconfig = None
        self.grabber = None
        
    def open(self):
        self.cameraconfig = CameraConfig('cameraonfig.xml')
        self.grabber = cwipc.realsense2.cwipc_realsense2()
        # May need to grab a few combined pointclouds and throw them away
        for i in range(SKIP_FIRST_GRABS):
            pc = self.grabber.get()
            pc.free()
        # Now we should update cameraconfig, if needed
        if self.cameraconfig.getserials() == ["0"]:
            # Default config file just created by us. Overwrite
            serials = self.getserials()
            self.cameraconfig.setserial(0, serials[0])
            for sn in serials[1:]:
                self.cameraconfig.addcamera(sn)
        else:
            # pre-existing. Check that config file and hardware setup match
            hwSerials = self.getserials()
            fileSerials = self.cameraconfig.getserials()
            ok = True
            for sn in hwSerials:
                if not sn in fileSerials:
                    ok = False
                    print(f'Camera {sn} is attached but not in cameraconfig.xml')
            for sn in fileSerials:
                if not sn in cameraSerials:
                    ok = False
                    print(f'Camera {sn} is in cameraconfig.xml but not attached')
            if not ok:
                print('Use --clean to calibrate this new hardware setup (or attach the right cameras)')
                sys.exit(1)
   
    def __del__(self):
        if self.grabber: self.grabber.free()
        self.grabber = None
    
    def getserials(self):
        """Get serial numbers of cameras"""
        rv = []
        ntile = self.grabber.maxtile()
        for i in range(ntile):
            info = self.grabber.get_tileinfo_raw(i)
            if info.camera != None:
                cam_id = info.camera
                cam_id = cam_id.decode('ascii')
                print('Found camera at tile', i, ', camera serial', cam_id)
                rv.append(cam_id)
        return rv
        
    def getmatrix(self, tilenum):
        return self.cameraconfig.getmatrix()
        
    def getcount(self):
        # Get the number of cameras and their tile numbers
        tiles = []
        maxtile = self.grabber.maxtile()
        if DEBUG: print('maxtile', maxtile)
        if maxtile == 1:
            return 1
        else:
            for i in range(1, maxtile):
                info = self.grabber.get_tileinfo_dict(i)
                if DEBUG: print('info', i, info)
                if info != None and info['ncamera'] == 1:
                    tiles.append(i)
        # Check that the tile numbers or a bitmap, as we expect (in join, for example)
        for i in range(len(tiles)):
            assert tiles[i] == (1<<i)
        return len(tiles)
        
    def getpointcloud(self):
        pc = self.grabber.get()
        assert pc
        return Pointcloud.from_cwipc(pc)