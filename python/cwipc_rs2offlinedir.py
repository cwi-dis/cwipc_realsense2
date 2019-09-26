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
        
    def getDepthData_notworking(self, filename):
        depthImage = loadImage(filename)
        depthImage = Image.fromarray(numpy.array(depthImage).astype("uint16"))
        assert depthImage.size == (640, 480)
        assert depthImage.mode == "I;16"
        depthData = depthImage.tobytes()
        assert len(depthData) == 640*480*2
        return depthData
        
    def getDepthData(self, filename):
        fp = open(filename)
        fp.readline()
        fp.readline()
        fp.readline()
        fp.readline()
        numbers = fp.read().split()
        assert(len(numbers)) == 640*480
        numbers = list(map(int, numbers))
        array = numpy.array(numbers)
        array = array.reshape((480, 640))
        depthImage = Image.fromarray(array.astype("uint16"))
        assert depthImage.size == (640, 480)
        assert depthImage.mode == "I;16"
        depthData = depthImage.tobytes()
        assert len(depthData) == 640*480*2
        return depthData
        
    def get(self):
        images = self.curData()
        gotPC = False
        while not gotPC:
            for i in range(len(images)):
                colorData, depthData = images[i]
                print(f'\t{i}: {len(colorData)} {len(depthData)}')
                self.converter.feed(i, colorData, depthData)
                gotPC = self.grabber.available(False)
                if not gotPC:
                    print('\t no data try again')
        rv = self.grabber.get()
        self.next()
        return rv
        
                

def main():
    certhDir = sys.argv[1]
    outputDir = sys.argv[2]
    dirHandler = CerthDir(certhDir)
    print(f'cameras: {dirHandler.cameras()}')
    while not dirHandler.eof():
        pc = dirHandler.get()
        print('got pointcloud')
        outputFile = os.path.join(outputDir, 'cloud-%d.ply' % dirHandler.index)
        cwipc.cwipc_write(outputFile, pc)
        pc.free()
    
if __name__ == '__main__':
    main()
    
