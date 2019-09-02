import sys
import cwipc
import cwipc_realsense2_rs2offline
from PIL import Image
import numpy

def loadImage(filename):
    return Image.open(filename)
    
def main():
    configFile = sys.argv[1]
    depthFile = sys.argv[2]
    colorFile = sys.argv[3]
    outputFile = sys.argv[4]
    converter = cwipc_realsense2_rs2offline.cwipc_rs2offline(configFile)
    grabber = converter.get_source()
    depthImage = loadImage(depthFile)
    depthImage = Image.fromarray(numpy.array(depthImage).astype("uint16"))
    assert depthImage.size == (640, 480)
    assert depthImage.mode == "I;16"
    colorImage = loadImage(colorFile)
    assert colorImage.size == (640, 480)
    assert colorImage.mode == "RGB"
    gotPC = False
    # Convert to bytes
    depthData = depthImage.tobytes()
    if 0 and len(depthData) == 640*480*4:
        newData = b""
        for i in range(0, len(depthData), 4):
            newData +=  depthData[i+1:i+3]
        depthData = newData
    assert len(depthData) == 640*480*2
#        depthData = b""
    open('tmpdump.bin', 'wb').write(depthData)
    colorData = colorImage.tobytes()
    assert len(colorData) in {640*480*3, 640_480*4}
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
    
