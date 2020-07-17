import xml.etree.ElementTree as ET

class CameraConfig:

    def __init__(self, confFilename):
        self.confFilename = confFilename
        self.serials = []
        self.matrices = []
        self.bbox = None
        self._readConf(self.confFilename)
        self._parseConf()
        
    def _readConf(self, confFilename):
        self.tree = ET.parse(confFilename)
        
    def _parseConf(self):
        root = self.tree.getroot()
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
        
    def save(self):
        assert 0
        
    def getcount(self):
        return len(self.serials)
        
    def getserials(self):
        return self.serials
        
    def getmatrix(self, tilenum):
        return self.matrices[tilenum]
        
    def addcamera(self, serial):
        assert 0
        
    def setmatrix(self, tilenum, matrix):
        assert 0
        
    def setbbox(self, threshold_near, threshold_far, heignt_min, height_max):
        assert 0
