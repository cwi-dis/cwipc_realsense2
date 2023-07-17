import os
import ctypes
import ctypes.util
import warnings
from typing import Optional
from cwipc.util import CwipcError, CWIPC_API_VERSION, cwipc_tiledsource
from cwipc.util import cwipc_tiledsource_p
from cwipc.util import _cwipc_dll_search_path_collection

__all__ = [
    "cwipc_offline_camera_settings",
    "cwipc_offline_settings",
    "RS2_FORMAT_RGB8",
    "RS2_FORMAT_Z16",
    "cwipc_realsense2",
    "cwipc_rs2offline",
    "cwipc_realsense2_dll_load"
]

#
# This is a workaround for the change in DLL loading semantics on Windows since Python 3.8
# Python no longer uses the PATH environment variable to load dependent dlls but only
# its own set. For that reason we list here a set of dependencies that we know are needed,
# find those on PATH, and add the directories where those DLLs are located while loading our
# DLL.
# The list does not have to be complete, as long as at least one DLL from each directory needed
# is listed.
# Dependencies of cwipc_util are automatically added.
# NOTE: this list must be kept up-to-date otherwise loading DLLs will fail with
# an obscure message "Python could not find module .... or one of its dependencies"
#
_WINDOWS_NEEDED_DLLS=[ # NOT USED AT THE TIME. CAUSING DLL Loading problems
    "realsense2",
]

class cwipc_offline_camera_settings(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("bpp", ctypes.c_int),
        ("fps", ctypes.c_int),
        ("format", ctypes.c_int)
    ]
    
class cwipc_offline_settings(ctypes.Structure):
    _fields_ = [
        ("color", cwipc_offline_camera_settings),
        ("depth", cwipc_offline_camera_settings),
    ]
class cwipc_offline_p(ctypes.c_void_p):
    pass
    
_cwipc_realsense2_dll_reference = None

#
# NOTE: the signatures here must match those in cwipc_util/api.h or all hell will break loose
#
def cwipc_realsense2_dll_load(libname : Optional[str]=None):
    """Load the cwipc_realsense2 DLL and assign the signatures (if not already loaded).
    
    If you want to load a non-default native library (for example to allow debugging low level code)
    call this method early, before any other method from this package.
    """
    global _cwipc_realsense2_dll_reference
    if _cwipc_realsense2_dll_reference: return _cwipc_realsense2_dll_reference
    
    with _cwipc_dll_search_path_collection(None) as loader:
        if libname == None:
            libname = 'cwipc_realsense2'
        if not os.path.isabs(libname):
            libname = loader.find_library(libname)
            if not libname:
                raise RuntimeError('Dynamic library realsense2 not found')
        assert libname
        _cwipc_realsense2_dll_reference = ctypes.CDLL(libname)
        if not _cwipc_realsense2_dll_reference:
            raise RuntimeError(f'Dynamic library {libname} cannot be loaded')
    
    _cwipc_realsense2_dll_reference.cwipc_realsense2.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_char_p), ctypes.c_ulong]
    _cwipc_realsense2_dll_reference.cwipc_realsense2.restype = cwipc_tiledsource_p
    if hasattr(_cwipc_realsense2_dll_reference, 'cwipc_rs2offline'):
        _cwipc_realsense2_dll_reference.cwipc_rs2offline.argtypes = [cwipc_offline_settings, ctypes.c_char_p, ctypes.POINTER(ctypes.c_char_p), ctypes.c_ulong]
        _cwipc_realsense2_dll_reference.cwipc_rs2offline.restype = cwipc_offline_p
        _cwipc_realsense2_dll_reference.cwipc_offline_free.argtypes = [cwipc_offline_p]
        _cwipc_realsense2_dll_reference.cwipc_offline_free.restype = None
        _cwipc_realsense2_dll_reference.cwipc_offline_get_source.argtypes = [cwipc_offline_p]
        _cwipc_realsense2_dll_reference.cwipc_offline_get_source.restype = cwipc_tiledsource_p
        _cwipc_realsense2_dll_reference.cwipc_offline_feed.argtypes = [cwipc_offline_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t]
        _cwipc_realsense2_dll_reference.cwipc_offline_feed.restype = ctypes.c_bool

    return _cwipc_realsense2_dll_reference

def RS2_FORMAT_RGB8():
    return ctypes.c_int.in_dll(cwipc_realsense2_dll_load(), "CWIPC_RS2_FORMAT_RGB8")
    
def RS2_FORMAT_Z16():
    return ctypes.c_int.in_dll(cwipc_realsense2_dll_load(), "CWIPC_RS2_FORMAT_Z16")
    
class cwipc_offline_wrapper:
    _cwipc_offline : Optional[cwipc_offline_p]

    def __init__(self, _cwipc_offline : Optional[cwipc_offline_p]):
        if _cwipc_offline != None:
            assert isinstance(_cwipc_offline, cwipc_offline_p)
        self._cwipc_offline = _cwipc_offline
        
    def _as_cwipc_offline_p(self) -> cwipc_offline_p:
        assert self._cwipc_offline
        return self._cwipc_offline
        
    def free(self) -> None:
        if self._cwipc_offline:
            cwipc_realsense2_dll_load().cwipc_offline_free(self._as_cwipc_offline_p())
        self._cwipc_offline = None

    def get_source(self) -> cwipc_tiledsource:
        """Returns the cwipc_tiledsource that can be used to read pointclouds from this cwipc_offline"""
        obj = cwipc_realsense2_dll_load().cwipc_offline_get_source(self._as_cwipc_offline_p())
        return cwipc_tiledsource(obj)
        
    def feed(self, camNum : int, frameNum : int, colorBuffer : bytearray | bytes | ctypes.Array[ctypes.c_char], depthBuffer : bytearray | bytes | ctypes.Array[ctypes.c_char]):
        """Feed RGB and D frame data into the cwipc_offline"""
        colorLength = len(colorBuffer)
        if isinstance(colorBuffer, bytearray):
            colorBuffer = (ctypes.c_char * colorLength).from_buffer(colorBuffer)
        elif isinstance(colorBuffer, bytes):
            colorBuffer = (ctypes.c_char * colorLength).from_buffer_copy(colorBuffer)
        colorPtr = ctypes.cast(colorBuffer, ctypes.c_void_p)
        depthLength = len(depthBuffer)
        if isinstance(depthBuffer, bytearray):
            depthBuffer = (ctypes.c_char * depthLength).from_buffer(depthBuffer)
        elif isinstance(depthBuffer, bytes):
            depthBuffer = (ctypes.c_char * depthLength).from_buffer_copy(depthBuffer)
        depthPtr = ctypes.cast(depthBuffer, ctypes.c_void_p)
        rv = cwipc_realsense2_dll_load().cwipc_offline_feed(self._as_cwipc_offline_p(), camNum, frameNum, colorPtr, colorLength, depthPtr, depthLength)
        return rv
        
def cwipc_realsense2(conffile : Optional[str]=None) -> cwipc_tiledsource:
    """Returns a cwipc_source object that grabs from a realsense2 camera and returns cwipc object on every get() call."""
    errorString = ctypes.c_char_p()
    cconffile = None
    if conffile:
        cconffile = conffile.encode('utf8')
    rv = cwipc_realsense2_dll_load().cwipc_realsense2(cconffile, ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString and errorString.value and not rv:
        raise CwipcError(errorString.value.decode('utf8'))
    if errorString and errorString.value:
        warnings.warn(errorString.value.decode('utf8'))
    if rv:
        return cwipc_tiledsource(rv)
    raise CwipcError("cwipc_realsense2: no cwipc_tiledsource created, but no specific error returned from C library")

def cwipc_rs2offline(settings : cwipc_offline_settings, conffile : str):
    """Returns a cwipc_source object that grabs from a realsense2 camera and returns cwipc object on every get() call."""
    errorString = ctypes.c_char_p()
    cconffile = conffile.encode('utf8')
    rv = cwipc_realsense2_dll_load().cwipc_rs2offline(settings, cconffile, ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString and errorString.value and not rv:
        raise CwipcError(errorString.value.decode('utf8'))
    if errorString and errorString.value:
        warnings.warn(errorString.value.decode('utf8'))
    if rv:
        return cwipc_offline_wrapper(rv)
    raise CwipcError("cwipc_rs2offline: no cwipc_tiledsource created, but no specific error returned from C library")
