import sys
import os
import cwipc
import cwipc_realsense2_rs2offline
from PIL import Image
import numpy
import xml.etree.ElementTree as ET

def loadImage(filename):
    return Image.open(filename)
    
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
    def __init__(self, dirname):
        self.configFile = os.path.join(dirname, 'offlineconfig.xml')
        assert os.path.exists(self.configFile)
        self.cameraNames = cameraNamesFromXml(self.configFile)
        self.colorFiles = CerthImageDir(os.path.join(dirname, 'color'))
        self.depthFiles = CerthImageDir(os.path.join(dirname, 'depth'))
        self.index = 0
        self._eof = False
        self.converter = cwipc_realsense2_rs2offline.cwipc_rs2offline(self.configFile)
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
        assert colorImage.size == (640, 480)
        assert colorImage.mode in {"RGB", "RGBA"}
        colorData = colorImage.tobytes()
        assert len(colorData) == 640*480*3
        return colorData
        
    def getDepthData(self, filename):
        depthImage = loadImage(filename)
        depthImage = Image.fromarray(numpy.array(depthImage).astype("uint16"))
        assert depthImage.size == (640, 480)
        assert depthImage.mode == "I;16"
        depthData = depthImage.tobytes()
        assert len(depthData) == 640*480*2
        return depthData

def main():
    certhDir = sys.argv[1]
    dirHandler = CerthDir(certhDir)
    print(f'cameras: {dirHandler.cameras()}')
    while not dirHandler.eof():
        print(f'PC {dirHandler.index}:')
        images = dirHandler.curData()
        for i in range(len(images)):
            colorData, depthData = images[i]
            print(f'\t{i}: {len(colorData)} {len(depthData)}')
        dirHandler.next()
     
def oldmain():
    configFile = sys.argv[1]
    depthFile = sys.argv[2]
    colorFile = sys.argv[3]
    outputFile = sys.argv[4]
    #b, g, r = colorImage.split()
    #colorImage = Image.merge("RGB", (r, g, b))
    gotPC = False
    # Convert to bytes
#        depthData = b""
    open('tmpdump.bin', 'wb').write(depthData)
    print(f"depth {len(depthData)} bytes, color {len(colorData)} bytes")
    # We need to feed the first image a couple of times, until the syncer gets the idea.
    while not gotPC:
        converter.feed(0, colorData, depthData)
        gotPC = grabber.available(False)
    pc = grabber.get()
    cwipc.cwipc_write(outputFile, pc)
    pc.free()
    grabber.free()
    
if __name__ == '__main__':
    main()
    
