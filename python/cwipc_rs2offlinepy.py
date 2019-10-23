import sys
import cwipc
import cwipc.realsense2
from PIL import Image
import numpy

def loadImage(filename):
    return Image.open(filename)
    
def main():
    if len(sys.argv) != 5:
        print(f"Usage: {sys.argv[0]} configFile colorFile depthFile outputFile", file=sys.stderr)
        sys.exit(1)
    configFile = sys.argv[1]
    colorFile = sys.argv[2]
    depthFile = sys.argv[3]
    outputFile = sys.argv[4]
    settings = cwipc.realsense2.cwipc_offline_settings(
        color=cwipc.realsense2.cwipc_offline_camera_settings(
            width=640,
            height=480,
            bpp=3,
            fps=60,
            format=cwipc.realsense2.RS2_FORMAT_RGB8()
        ),
        depth=cwipc.realsense2.cwipc_offline_camera_settings(
            width=640,
            height=480,
            bpp=2,
            fps=60,
            format=cwipc.realsense2.RS2_FORMAT_Z16()
        ),
        
    )
    converter = cwipc.realsense2.cwipc_rs2offline(settings, configFile)
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
    converter.feed(0, frameNum, colorData, depthData)
    assert grabber.available(True)
    pc = grabber.get()
    assert pc
    assert pc.get_uncompressed_size()
    cwipc.cwipc_write(outputFile, pc)
    pc.free()
    grabber.free()
    
if __name__ == '__main__':
    main()
    
