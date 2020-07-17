import sys
import os
import cwipc

from .pointcloud import Pointcloud
from .cameraconfig import CameraConfig

class FileGrabber:
    def __init__(self, dirname):
        self.pcFilename = os.path.join(dirname, "cwipc_calibrate_calibrated.ply")
        confFilename = os.path.join(dirname, "cameraconfig.xml")
        self.config = CameraConfig(confFilename)
        
    def getcount(self):
        return self.config.getcount()
        
    def getserials(self):
        return self.config.getserials()
        
    def getmatrix(self, tilenum):
        return self.config.getmatrix(tilenum)
        
    def getpointcloud(self):
        pc = cwipc.cwipc_read(self.pcFilename, 0)
        return Pointcloud.from_cwipc(pc)
