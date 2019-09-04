import sys
import os
import json
import xml.etree.ElementTree as ET

BASECONFIG="""<?xml version="1.0" ?>
<file>
    <CameraConfig>
        <system usb2width="640" usb2height="480" usb2fps="15" usb3width="1280" usb3height="720" usb3fps="30" />
        <postprocessing depthfiltering="0" backgroundremoval="0" greenscreenremoval="0" cloudresolution="0" tiling="0" tilingresolution="0.01" tilingmethod="camera">
            <depthfilterparameters decimation_value="2" spatial_iterations="4" spatial_alpha="0.25" spatial_delta="30" spatial_filling="0" temporal_alpha="0.4" temporal_delta="20" temporal_percistency="3" />
        </postprocessing>
    </CameraConfig>
</file>
"""

def baseConfig():
    tree = ET.ElementTree(ET.fromstring(BASECONFIG))
    return tree
    
def fillTrafo(elt, items, transpose=False, translateScaling=1):
    """Fill and XML element with a 4x4 matrix based on 12 values specifying a 3x3 matrix and a vector"""
    if transpose:
        elt.set('v00', str(items[0]))
        elt.set('v01', str(items[1]))
        elt.set('v02', str(items[2]))
        elt.set('v03', str(items[9]*translateScaling)) # Translation
    
        elt.set('v10', str(items[3]))
        elt.set('v11', str(items[4]))
        elt.set('v12', str(items[5]))
        elt.set('v13', str(items[10]*translateScaling)) # Translation
    
        elt.set('v20', str(items[6]))
        elt.set('v21', str(items[7]))
        elt.set('v22', str(items[8]))
        elt.set('v23', str(items[11]*translateScaling)) # Translation
    else:
        elt.set('v00', str(items[0]))
        elt.set('v01', str(items[3]))
        elt.set('v02', str(items[6]))
        elt.set('v03', str(items[9]*translateScaling)) # Translation
    
        elt.set('v10', str(items[1]))
        elt.set('v11', str(items[4]))
        elt.set('v12', str(items[7]))
        elt.set('v13', str(items[10]*translateScaling)) # Translation
    
        elt.set('v20', str(items[2]))
        elt.set('v21', str(items[5]))
        elt.set('v22', str(items[8]))
        elt.set('v23', str(items[11]*translateScaling)) # Translation
    
    elt.set('v30', '0')
    elt.set('v31', '0')
    elt.set('v32', '0')
    elt.set('v33', '1')
    
def entry2xml(entry):
    rv = ET.Element('camera')
    rv.set('serial', entry['name'])
    trafoParent = ET.SubElement(rv, 'trafo')
    trafo = ET.SubElement(trafoParent, 'values')
    fillTrafo(trafo, entry['cam2worldTrafo'], transpose=True, translateScaling=1)
    intParent = ET.SubElement(rv, 'intrinsicTrafo')
    intTrafo = ET.SubElement(intParent, 'values')
    fillTrafo(intTrafo, entry['color2depthTrafo'], transpose=True)
    return rv

def indentXml(elem, level=0):
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indentXml(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i
            
def readTrafo(filename):
    """Read a file with 12 floating point numbers (signifying rotation matrix and translation vector)"""
    data = open(filename).read()
    entries = data.split()
    rv = []
    for entry in entries:
        entry = entry.strip()
        if entry:
            rv.append(float(entry.strip()))
    assert(len(rv) == 12)
    return rv
    
def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} certh-dir outputfile.xml", file=sys.stderr)
    certhDir = sys.argv[1]
    outputFilename = sys.argv[2]
    repoFilename = os.path.join(certhDir, 'device_repository_vrt.json')
    deviceList = json.load(open(repoFilename))
    entries = []
    for device in deviceList:
        entry = {}
        entry['name'] = device['Device']
        trafo = device['Color Depth Rotation'] + device['Color Depth Translation']
        assert(len(trafo) == 12)
        entry['color2depthTrafo'] = trafo
        trafoFilename = os.path.join(certhDir, '%s.extrinsics' % entry['name'])
        entry['cam2worldTrafo'] = readTrafo(trafoFilename)
        entries.append(entry)
    configDoc = baseConfig()
    root = configDoc.getroot()
    config = root.find('CameraConfig')
    for entry in entries:
        elt = entry2xml(entry)
        config.append(elt)
        indentXml(root)
    configDoc.write(outputFilename)
    print(f'{outputFilename}: {len(entries)} cameras created')
    
if __name__ == '__main__':
    main()
    
