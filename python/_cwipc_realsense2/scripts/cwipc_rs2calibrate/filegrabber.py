import sys
import os
import cwipc
import xml.etree.ElementTree as ET

from .pointcloud import Pointcloud

class FileGrabber:
    def __init__(self, dirname):
        self.pcFilename = os.path.join(dirname, "cwipc_calibrate_calibrated.ply")
        confFilename = os.path.join(dirname, "cameraconfig.xml")
        self.serials = []
        self.matrices = []
        self._parseConf(confFilename)
        
    def _parseConf(self, confFilename):
        tree = ET.parse(confFilename)
        root = tree.getroot()
        for camElt in root.findall('CameraConfig/camera'):
            serial = camElt.attrib['serial']
            assert serial
            trafoElts = list(camElt.iter('trafo'))
            assert len(trafoElts) == 1
            trafoElt = trafoElts[0]
            valuesElts = list(trafoElt.iter('values'))
            assert len(valuesElts) == 1
            valuesElt = valuesElts[0]
            va = valuesElt.attrib
            trafo = [
                [float(va['v00']), float(va['v01']), float(va['v02']), float(va['v03'])],
                [float(va['v10']), float(va['v11']), float(va['v12']), float(va['v13'])],
                [float(va['v20']), float(va['v21']), float(va['v22']), float(va['v23'])],
                [float(va['v30']), float(va['v31']), float(va['v32']), float(va['v33'])],
            ]
            self.serials.append(serial)
            self.matrices.append(trafo)
        # import pdb ; pdb.set_trace()
        
    def getcount(self):
        return len(self.serials)
        
    def getserials(self):
        return self.serials
        
    def getmatrix(self, tilenum):
        return self.matrices[tilenum]
        
    def getpointcloud(self):
        pc = cwipc.cwipc_read(self.pcFilename, 0)
        return Pointcloud.from_cwipc(pc)
