import sys
import cwipc
import cwipc_realsense2_rs2offline
from PIL import Image
import numpy

def loadImage(filename):
    return Image.open(filename)
    
def main():
    configFile = sys.argv[1]
    colorFile = sys.argv[2]
    depthFile = sys.argv[3]
    outputFile = sys.argv[4]
    cwipc_realsense2_rs2offline.initconsts()
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
    converter = cwipc_realsense2_rs2offline.cwipc_rs2offline(settings, configFile)
    grabber = converter.get_source()
    depthImage = loadImage(depthFile)
    depthImage = Image.fromarray(numpy.array(depthImage).astype("uint16"))
    assert depthImage.size == (settings.depth.width, settings.depth.height)
    assert depthImage.mode == "I;16"
    colorImage = loadImage(colorFile)
    assert colorImage.size == (settings.color.width, settings.color.height)
    assert colorImage.mode == "RGB"
    gotPC = False
    # Convert to bytes
    depthData = depthImage.tobytes()
    assert len(depthData) == settings.depth.width*settings.depth.height*settings.depth.bpp

    colorData = colorImage.tobytes()
    assert len(colorData) == settings.color.width*settings.color.height*settings.color.bpp

    print(f"depth {len(depthData)} bytes, color {len(colorData)} bytes")

    # We need to feed the first image a couple of times, until the syncer gets the idea.
    frameNum = 0
    while not gotPC:
        converter.feed(0, frameNum, colorData, depthData)
        gotPC = grabber.available(False)
        frameNum += 1
    pc = grabber.get()
    cwipc.cwipc_write(outputFile, pc)
    pc.free()
    grabber.free()
    
if __name__ == '__main__':
    main()
    
