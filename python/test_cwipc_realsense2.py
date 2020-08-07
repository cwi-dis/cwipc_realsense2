import unittest
import cwipc
import _cwipc_realsense2
import os
import sys
import tempfile

if 0:
    # This code can be used to debug the C++ code in XCode:
    # - build for XCode with cmake
    # - build the cwipc_util project
    # - Fix the pathname for the dylib
    # - run `python3 test_cwipc_util`
    # - Attach to python in the XCode debugger
    # - press return to python3.
    import _cwipc_realsense2
    #_cwipc_realsense2._cwipc_realsense2_dll('/Users/jack/src/VRTogether/cwipc_realsense2/build-xcode/lib/Debug/libcwipc_realsense2.dylib')
    _cwipc_realsense2._cwipc_realsense2_dll('C:/Users/vrtogether/src/VRtogether/cwipc_realsense2/build/bin/RelWithDebInfo/cwipc_realsense2.dll')
    print('Type return after attaching in XCode debugger (pid=%d) - ' % os.getpid())
    sys.stdin.readline()

#
# Windows search path is horrible. Work around it for testing with an environment variable
#
if 'CWIPC_TEST_DLL' in os.environ:
    filename = os.environ['CWIPC_TEST_DLL']
    dllobj = _cwipc_realsense2._cwipc_realsense2_dll(filename)

#
# Find directories for test inputs and outputs
#
_thisdir=os.path.dirname(os.path.join(os.getcwd(), __file__))
_topdir=os.path.dirname(_thisdir)
TEST_FIXTURES_DIR=os.path.join(_topdir, "tests", "fixtures")
print('xxxjack', _thisdir, _topdir, TEST_FIXTURES_DIR)
TEST_OUTPUT_DIR=os.path.join(TEST_FIXTURES_DIR, "output")
if not os.access(TEST_OUTPUT_DIR, os.W_OK):
    TEST_OUTPUT_DIR=tempfile.mkdtemp('cwipc_realsense2_test')

class TestApi(unittest.TestCase):
    
    def _open_grabber(self):
        try:
            grabber = _cwipc_realsense2.cwipc_realsense2()
        except cwipc.CwipcError as arg:
            if str(arg) == 'cwipc_realsense2: no realsense cameras found':
                self.skipTest(str(arg))
            raise
        return grabber
        
    def test_cwipc_realsense2(self):
        """Test that we can grab a realsense2 image"""
        grabber = None
        pc = None
        try:
            grabber = self._open_grabber()
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
            grabber = self._open_grabber()
            nTile = grabber.maxtile()
            self.assertGreaterEqual(nTile, 1)
            # Assure the non-tiled-tile exists and points nowhere.
            tileInfo = grabber.get_tileinfo_dict(0)
            self.assertIn('normal', tileInfo)
            self.assertIn('camera', tileInfo)
            self.assertIn('ncamera', tileInfo)
            # Untrue if multiple realsenses connected: self.assertLessEqual(tileInfo['ncamera'], 1)
            # Test some minimal conditions for other tiles
            for i in range(1, nTile):
                tileInfo = grabber.get_tileinfo_dict(i)
                if i in (1, 2, 4, 8, 16, 32, 64, 128):
                    # These tiles should exist and have a normal and camera ID (which may be None)
                    self.assertIn('normal', tileInfo)
                    self.assertIn('camera', tileInfo)
        finally:
            if grabber: grabber.free()

    def test_cwipc_rs2offline(self):
        """Test that we can create a pointcloud from offline images"""
        if sys.platform == 'linux':
            self.skipTest('rs2offline destructor will hang on Linux')
        grabber = None
        pc = None
        try:
            conffile = os.path.join(TEST_FIXTURES_DIR, 'input', 'offlineconfig.xml')
            settings = _cwipc_realsense2.cwipc_offline_settings(
                color=_cwipc_realsense2.cwipc_offline_camera_settings(
                    width=640,
                    height=480,
                    bpp=3,
                    fps=60,
                    format=_cwipc_realsense2.RS2_FORMAT_RGB8()
                ),
                depth=_cwipc_realsense2.cwipc_offline_camera_settings(
                    width=640,
                    height=480,
                    bpp=2,
                    fps=60,
                    format=_cwipc_realsense2.RS2_FORMAT_Z16()
                ),
        
            )
            converter = _cwipc_realsense2.cwipc_rs2offline(settings, conffile)
            grabber = converter.get_source()
            self.assertFalse(grabber.eof())
            self.assertFalse(grabber.available(True))
        finally:
            if grabber: grabber.free()
            if pc: pc.free()


    def _verify_pointcloud(self, pc):
        points = pc.get_points()
        self.assertGreater(len(points), 1)
        halfway = int((len(points)+1)/2)
        p0 = points[0].x, points[0].y, points[0].z, points[0].r, points[0].g, points[0].b
        p1 = points[halfway].x, points[halfway].y, points[halfway].z, points[halfway].r, points[halfway].g, points[halfway].b
        self.assertNotEqual(p0, p1)
   
if __name__ == '__main__':
    unittest.main()
