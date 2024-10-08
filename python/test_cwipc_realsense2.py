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
    _cwipc_realsense2.cwipc_realsense2_dll_load('C:/Users/vrtogether/src/VRtogether/cwipc_realsense2/build/bin/RelWithDebInfo/cwipc_realsense2.dll')
    print('Type return after attaching in XCode debugger (pid=%d) - ' % os.getpid())
    sys.stdin.readline()

#
# Windows search path is horrible. Work around it for testing with an environment variable
#
if 'CWIPC_TEST_DLL' in os.environ:
    filename = os.environ['CWIPC_TEST_DLL']
    dllobj = _cwipc_realsense2.cwipc_realsense2_dll_load(filename)

#
# Find directories for test inputs and outputs
#
_thisdir=os.path.dirname(os.path.join(os.getcwd(), __file__))
_topdir=os.path.dirname(_thisdir)
TEST_FIXTURES_DIR=os.path.join(_topdir, "tests", "fixtures")
TEST_FIXTURES_PLAYBACK_CONFIG=os.path.join(TEST_FIXTURES_DIR, "input", "recording", "cameraconfig.json")
TEST_OUTPUT_DIR=os.path.join(TEST_FIXTURES_DIR, "output")
if not os.access(TEST_OUTPUT_DIR, os.W_OK):
    TEST_OUTPUT_DIR=tempfile.mkdtemp('cwipc_realsense2_test')  # type: ignore

class TestApi(unittest.TestCase):
    
    def _open_grabber(self):
        try:
            grabber = _cwipc_realsense2.cwipc_realsense2("auto")
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
            self.assertIsNotNone(pc)
            assert pc # Only to keep linters happy
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
            self.assertIsNotNone(tileInfo)
            assert tileInfo # Only to keep linters happy
            self.assertIn('normal', tileInfo)
            self.assertIn('cameraName', tileInfo)
            self.assertIn('cameraMask', tileInfo)
            self.assertIn('ncamera', tileInfo)
            # Untrue if multiple realsenses connected: self.assertLessEqual(tileInfo['ncamera'], 1)
            # Test some minimal conditions for other tiles
            for i in range(1, nTile):
                tileInfo = grabber.get_tileinfo_dict(i)
                self.assertIsNotNone(tileInfo)
                assert tileInfo # Only to keep linters happy
                if i in (1, 2, 4, 8, 16, 32, 64, 128):
                    # These tiles should exist and have a normal and camera ID (which may be None)
                    self.assertIn('normal', tileInfo)
                    self.assertIn('camera', tileInfo)
        finally:
            if grabber: grabber.free()

    def test_cwipc_realsense2_playback(self):
        """Test that we can grab a realsense2 image from the playback grabber"""
        grabber = None
        pc = None
        try:
            grabber = _cwipc_realsense2.cwipc_realsense2_playback(TEST_FIXTURES_PLAYBACK_CONFIG)
            self.assertFalse(grabber.eof())
            self.assertTrue(grabber.available(True))
            pc = grabber.get()
            # Hack around problem: sometimes we get NULL or empty clouds at the beginnging of the run.
            if pc is None or pc.count() == 0:
                if pc != None:
                    pc.free()
                self.assertTrue(grabber.available(True))
                pc = grabber.get()
            if pc is None or pc.count() == 0:
                if pc != None:
                    pc.free()
                self.assertTrue(grabber.available(True))
                pc = grabber.get()
            self.assertIsNotNone(pc)
            assert pc # Only to keep linters happy
            self._verify_pointcloud(pc)
        finally:
            if grabber: grabber.free()
            if pc: pc.free()

    def _verify_pointcloud(self, pc : cwipc.cwipc_wrapper):
        points = pc.get_points()
        self.assertGreater(len(points), 1)
        halfway = int((len(points)+1)/2)
        p0 = points[0].x, points[0].y, points[0].z, points[0].r, points[0].g, points[0].b
        p1 = points[halfway].x, points[halfway].y, points[halfway].z, points[halfway].r, points[halfway].g, points[halfway].b
        self.assertNotEqual(p0, p1)
        