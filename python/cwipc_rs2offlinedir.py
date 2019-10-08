import sys
import os
import cwipc
import cwipc_realsense2_rs2offline
from PIL import Image
import numpy
import xml.etree.ElementTree as ET

def loadImage(filename):
    return Image.open(filename)
    
def settings(which):
    cwipc_realsense2_rs2offline.initconsts()
    if which == 'prod':
        settings = cwipc_realsense2_rs2offline.cwipc_offline_settings(
            color=cwipc_realsense2_rs2offline.cwipc_offline_camera_settings(
                width=1280,
                height=720,
                bpp=3,
                fps=60,
                format=cwipc_realsense2_rs2offline.RS2_FORMAT_RGB8
            ),
            depth=cwipc_realsense2_rs2offline.cwipc_offline_camera_settings(
                width=320,
                height=180,
                bpp=2,
                fps=60,
                format=cwipc_realsense2_rs2offline.RS2_FORMAT_Z16
            ),
        
        )
        return settings
    elif which == 'test':
        settings = cwipc_realsense2_rs2offline.cwipc_offline_settings(
            color=cwipc_realsense2_rs2offline.cwipc_offline_camera_settings(
                width=640,
                height=480,
                bpp=3,
                fps=60,
                format=cwipc_realsense2_rs2offline.RS2_FORMAT_RGB8
            ),
            depth=cwipc_realsense2_rs2offline.cwipc_offline_camera_settings(
                width=640,
                height=480,
                bpp=2,
                fps=60,
                format=cwipc_realsense2_rs2offline.RS2_FORMAT_Z16
            ),
        
        )
        return settings
    else:
        assert False, 'settings argument must be "test" or "prod", or else edit source'
        
def cameraNamesFromXml(filename):
    """Parse a cameraconfig file and return a list of camera names"""
    tree = ET.parse(filename)
    root = tree.getroot()
    rv = []
    for elt in root.findall('.//camera'):
        rv.append(elt.get('serial'))
    return rv
    
class CerthImageDir:
    def __init__(self, dirName):
        self.dirName = dirName
        self.dirContents = os.listdir(self.dirName)
        
    def getFilename(self, camName, index):
        pattern = '%d_%s_' % (index, camName)
        candidates = filter(lambda fn: fn.startswith(pattern), self.dirContents)
        candidates = list(candidates)
        if len(candidates) == 1:
            return os.path.join(self.dirName, candidates[0])
        if len(candidates) > 1:
            print(f'Multiple files: {candidates}')
        return None
    

class CerthDir:
    def __init__(self, dirname, captureType):
        self.configFile = os.path.join(dirname, 'offlineconfig.xml')
        assert os.path.exists(self.configFile)
        self.offlineSettings = settings(captureType)
        self.cameraNames = cameraNamesFromXml(self.configFile)
        self.colorFiles = CerthImageDir(os.path.join(dirname, 'color'))
        self.depthFiles = CerthImageDir(os.path.join(dirname, 'depth'))
        self.index = 0
        self._eof = False
        self.converter = cwipc_realsense2_rs2offline.cwipc_rs2offline(self.offlineSettings, self.configFile)
        self.grabber = self.converter.get_source()

    def cameras(self):
        return self.cameraNames
        
    def _curFilenames(self):
        rv = []
        any = False
        for camName in self.cameraNames:
            colorFilename = self.colorFiles.getFilename(camName, self.index)
            if colorFilename:
                any = True
            depthFilename = self.depthFiles.getFilename(camName, self.index)
            if depthFilename:
                any = True
            rv.append((colorFilename, depthFilename))
        if not any:
            self._eof = True
        return rv
        
    def curData(self):
        rv = []
        any = False
        for camName in self.cameraNames:
            colorFilename = self.colorFiles.getFilename(camName, self.index)
            if colorFilename:
                any = True
                colorData = self.getColorData(colorFilename)
            else:
                colorData = None
            depthFilename = self.depthFiles.getFilename(camName, self.index)
            if depthFilename:
                any = True
                depthData = self.getDepthData(depthFilename)
            else:
                depthData = None
            rv.append((colorData, depthData))
        if not any:
            self._eof = True
        return rv

    def next(self):
        rv = self._curFilenames()
        self.index += 1
        return rv
       
    def eof(self):
        return self._eof
        
    def getColorData(self, filename):
        colorImage = loadImage(filename)
        assert colorImage.size == (self.offlineSettings.color.width, self.offlineSettings.color.height)
        assert colorImage.mode in {"RGB", "RGBA"}
        colorData = colorImage.tobytes()
        assert len(colorData) == self.offlineSettings.color.width*self.offlineSettings.color.height*self.offlineSettings.color.bpp
        return colorData
        
    def getDepthData_notworking(self, filename):
        depthImage = loadImage(filename)
        depthImage = Image.fromarray(numpy.array(depthImage).astype("uint16"))
        assert depthImage.size == (self.offlineSettings.depth.width, self.offlineSettings.depth.height)
        assert depthImage.mode == "I;16"
        depthData = depthImage.tobytes()
        assert len(depthData) == self.offlineSettings.depth.width*self.offlineSettings.depth.height*self.offlineSettings.depth.bpp
        return depthData
        
    def getDepthData(self, filename):
        fp = open(filename)
        fp.readline()
        fp.readline()
        fp.readline()
        fp.readline()
        numbers = fp.read().split()
        assert(len(numbers)) == self.offlineSettings.depth.width*self.offlineSettings.depth.height
        numbers = list(map(int, numbers))
        array = numpy.array(numbers)
        array = array.reshape((self.offlineSettings.depth.height, self.offlineSettings.depth.width))
        depthImage = Image.fromarray(array.astype("uint16"))
        assert depthImage.size == (self.offlineSettings.depth.width, self.offlineSettings.depth.height)
        assert depthImage.mode == "I;16"
        depthData = depthImage.tobytes()
        assert len(depthData) == self.offlineSettings.depth.width*self.offlineSettings.depth.height*self.offlineSettings.depth.bpp
        return depthData
        
    def get(self):
        images = self.curData()
        for i in range(len(images)):
            colorData, depthData = images[i]
            self.converter.feed(i, self.index, colorData, depthData)
        gotPC = self.grabber.available(True)
        if not gotPC:
            print(f"Frame {self.index}: no pointcloud available")
        rv = self.grabber.get()
        self.next()
        return rv
        
                

def main():
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} certh-directory capturetype ply-output-directory")
        print("capturetype is prod (1280x720 RGB, 320x180 D) or test (640x480 RGB and D)")
        sys.exit(1)
    certhDir = sys.argv[1]
    certhCapturetype = sys.argv[2]
    outputDir = sys.argv[3]
    dirHandler = CerthDir(certhDir, certhCapturetype)
    print(f'cameras: {dirHandler.cameras()}')
    while not dirHandler.eof():
        print(f"Processing frame {dirHandler.index}")
        pc = dirHandler.get()
        outputFile = os.path.join(outputDir, 'cloud-%d.ply' % dirHandler.index)
        cwipc.cwipc_write(outputFile, pc)
        pc.free()
    
if __name__ == '__main__':
    main()
    
