import unittest
import cwipc
import cwipc.realsense2
import os
import sys
import tempfile

if 1:
    # This code can be used to debug the C++ code in XCode:
    # - build for XCode with cmake
    # - build the cwipc_util project
    # - Fix the pathname for the dylib
    # - run `python3 test_cwipc_util`
    # - Attach to python in the XCode debugger
    # - press return to python3.
    import cwipc.realsense2
    cwipc.realsense2._cwipc_realsense2_dll('/Users/jack/src/VRTogether/cwipc_realsense2/build-xcode/lib/Debug/libcwipc_realsense2.dylib')
    print('Type return after attaching in XCode debugger (pid=%d) - ' % os.getpid())
    sys.stdin.readline()

#
# Windows search path is horrible. Work around it for testing with an environment variable
#
if 'CWIPC_TEST_DLL' in os.environ:
	filename = os.environ['CWIPC_TEST_DLL']
	dllobj = cwipc.realsense2._cwipc_realsense2_dll(filename)
class TestApi(unittest.TestCase):
        
    def test_cwipc_realsense2(self):
        """Test that we can grab a realsense2 image"""
        grabber = None
        pc = None
        try:
            grabber = cwipc.realsense2.cwipc_realsense2()
            self.assertFalse(grabber.eof())
            self.assertTrue(grabber.available(True))
            pc = grabber.get()
            self._verify_pointcloud(pc)
        finally:
            if grabber: grabber.free()
            if pc: pc.free()

    def test_cwipc_realsense2_tileinfo(self):
        """Test that we can get tileinfo from a realsense2 grabber"""
        grabber = None
        try:
            grabber = cwipc.realsense2.cwipc_realsense2()
            nTile = grabber.maxtile()
            self.assertGreaterEqual(nTile, 1)
            # Assure the non-tiled-tile exists and points nowhere.
            tileInfo = grabber.get_tileinfo_dict(0)
            self.assertIn('normal', tileInfo)
            self.assertIn('camera', tileInfo)
            self.assertIn('ncamera', tileInfo)
            self.assertLessEqual(tileInfo['ncamera'], 1)
            # Test some minimal conditions for other tiles
            for i in range(1, nTile):
                tileInfo = grabber.get_tileinfo_dict(i)
                if i in (1, 2, 4, 8, 16, 32, 64, 128):
                    # These tiles should exist and have a camera ID
                    self.assertIn('normal', tileInfo)
                    self.assertNotEqual(tileinfo['camera'], None)
        finally:
            if grabber: grabber.free()

    def test_cwipc_realsense2_configfile(self):
        """Test that we can grab a realsense2 image when we pass in a (non-existent) config file"""
        grabber = None
        pc = None
        try:
            grabber = cwipc.realsense2.cwipc_realsense2("./nonexistent.xml")
            self.assertFalse(grabber.eof())
            self.assertTrue(grabber.available(True))
            pc = grabber.get()
            self._verify_pointcloud(pc)
        finally:
            if grabber: grabber.free()
            if pc: pc.free()

    def _verify_pointcloud(self, pc):
        points = pc.get_points()
        self.assertGreater(len(points), 1)
        p0 = points[0].x, points[0].y, points[0].z
        p1 = points[len(points)-1].x, points[len(points)-1].y, points[len(points)-1].z
        self.assertNotEqual(p0, p1)
   
if __name__ == '__main__':
    unittest.main()
