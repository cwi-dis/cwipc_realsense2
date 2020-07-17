import sys
import os
import numpy as np
import open3d

from .pointcloud import Pointcloud
from .ui import UI
DEBUG=False

CONFIGFILE="""<?xml version="1.0" ?>
<file>
    <CameraConfig>
        <system usb2width="640" usb2height="480" usb2fps="15" usb3width="1280" usb3height="720" usb3fps="30" />
        <postprocessing density="1" height_min="0" height_max="0" depthfiltering="1" backgroundremoval="0" greenscreenremoval="0" cloudresolution="0" tiling="0" tilingresolution="0.01" tilingmethod="camera">
            <depthfilterparameters {distance} decimation_value="1" spatial_iterations="4" spatial_alpha="0.25" spatial_delta="30" spatial_filling="0" temporal_alpha="0.4" temporal_delta="20" temporal_percistency="3" />
        </postprocessing>
        {cameras}
    </CameraConfig>
</file>
"""

CONFIGCAMERA="""
        <camera serial="{serial}" backgroundx="0" backgroundy="0" backgroundz="0">
            <trafo>
                {matrixinfo}
            </trafo>
        </camera>
"""

                    
class Calibrator:
    def __init__(self, distance, refpoints):
        self.ui = UI()
        self.cameraserial = []
        self.near = 0.5 * distance
        self.far = 2.0 * distance
        self.refpoints = refpoints
        self.grabber = None
        self.cameraserial = []
        self.pointclouds = []
        self.coarse_calibrated_pointclouds = []
        self.fine_calibrated_pointclouds = []
        self.refpointcloud = None
        self.coarse_matrix = []
        self.fine_matrix = []
        self.workdir = os.getcwd() # o3d visualizer can change directory??!?
        sys.stdout.flush()
        
    def __del__(self):
        self.grabber = None
        self.pointclouds = None
        self.coarse_calibrated_pointclouds = None
        self.refpointcloud = None
        
    def open(self, grabber, clean, reuse):
        if reuse:
            assert 0, "--reuse not yet implemented"
        elif clean:
            if os.path.exists('cameraconfig.xml'):
                os.unlink('cameraconfig.xml')
        if os.path.exists('cameraconfig.xml'):
            self.ui.show_error('%s: cameraconfig.xml already exists, please supply --clean or --reuse argument' % sys.argv[0])
            sys.exit(1)
        # Set initial config file, for filtering parameters
        self.writeconfig()
        self.grabber = grabber
        self.cameraserial = self.grabber.getserials()

    def grab(self, noinspect):
        if not self.cameraserial:
            self.ui.show_error('* No realsense cameras found')
            return False
        self.ui.show_message('* Grabbing pointclouds')
        self.get_pointclouds()
        if DEBUG:
            for i in range(len(self.pointclouds)):
                self.ui.show_message('Saving pointcloud {} to file'.format(i))
                self.pointclouds[i].save('pc-%d.ply' % i)
        #
        # First show the pointclouds for visual inspection.
        #
        if noinspect: return
        grab_ok = False
        while not grab_ok:
            sys.stdout.flush()
            for i in range(len(self.pointclouds)):
                self.ui.show_prompt(f'Showing grabbed pointcloud from camera {i} for visual inspection')
                self.ui.show_points(f'Inspect grab from {self.cameraserial[i]}', self.pointclouds[i], from000=True)
            grab_ok = self.ui.show_question('Can you select the balls on the cross from this pointcloud?', canretry=True)
            if not grab_ok:
                self.ui.show_message('* discarding 10 pointclouds')
                for i in range(10):
                    self.get_pointclouds()
                self.ui.show_message('* Grabbing pointclouds again')
                self.pointclouds = []
                self.get_pointclouds()
        
        
    def run_coarse(self):
        self.ui.show_prompt('Pick red, orange, yellow, blue points on reference image', isedit=True)
        #
        # Pick reference points
        #
        refpoints = self.ui.pick_points('Pick points on reference', self.refpointcloud)
        
        #
        # Pick points in images
        #
        for i in range(len(self.pointclouds)):
            matrix_ok = False
            while not matrix_ok:
                self.ui.show_prompt(f'Pick red, orange, yellow, blue points on camera {i} pointcloud', isedit=True)
                pc_refpoints = self.ui.pick_points(f'Pick points on {self.cameraserial[i]}', self.pointclouds[i], from000=True)
                info = self.align_pair(self.pointclouds[i], pc_refpoints, self.refpointcloud, refpoints, False)
                self.ui.show_prompt(f'Inspect resultant orientation of camera {i} pointcloud')
                new_pc = self.pointclouds[i].transform(info)
                self.ui.show_points(f'Result from {self.cameraserial[i]}', new_pc)
                matrix_ok = self.ui.show_question('Does that look good?', canretry=True)
            assert len(self.coarse_matrix) == i
            assert len(self.coarse_calibrated_pointclouds) == i
            self.coarse_matrix.append(info)
            self.coarse_calibrated_pointclouds.append(new_pc)
        #
        # Show result
        #
        self.ui.show_prompt('Inspect the resultant merged pointclouds of all cameras')
        joined = Pointcloud.from_join(*tuple(self.coarse_calibrated_pointclouds))
        os.chdir(self.workdir)
        joined.save('cwipc_calibrate_coarse.ply')
        self.ui.show_points('Inspect manual calibration result', joined)
        
    def skip_coarse(self):
        self.coarse_calibrated_pointclouds = self.pointclouds
        for i in range(len(self.cameraserial)):
            self.coarse_matrix.append([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ])
        
    def run_fine(self, bbox, correspondence):
        for i in range(len(self.cameraserial)):
            self.fine_matrix.append([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ])
        if bbox:
            # Apply bounding box to pointclouds
            for i in range(len(self.coarse_calibrated_pointclouds)):
                self.coarse_calibrated_pointclouds[i] = self.coarse_calibrated_pointclouds[i].bbox(bbox)
        if len(self.coarse_calibrated_pointclouds) <= 1:
            self.ui.show_message('* Skipping fine-grained calibration: only one camera')
            self.fine_calibrated_pointclouds = self.coarse_calibrated_pointclouds
            return
        joined = Pointcloud.from_join(*tuple(self.coarse_calibrated_pointclouds))
        self.ui.show_prompt('Inspect pointcloud after applying bounding box, before fine-grained calibration')
        self.ui.show_points('Inspect bounding box result', joined)
        # We will align everything to the first camera
        refPointcloud = self.coarse_calibrated_pointclouds[0]
        assert len(self.fine_calibrated_pointclouds) == 0
        self.fine_calibrated_pointclouds.append(refPointcloud)
        
        for i in range(1, len(self.coarse_calibrated_pointclouds)):
            srcPointcloud =  self.coarse_calibrated_pointclouds[i]
            newMatrix = self.align_fine(refPointcloud,srcPointcloud, correspondence)
            newPointcloud = srcPointcloud.transform(newMatrix)
            self.fine_calibrated_pointclouds.append(newPointcloud)
            self.fine_matrix[i] = newMatrix
            # print('xxxjack new matrix', newMatrix)
        self.ui.show_prompt('Inspect the resultant merged pointclouds of all cameras')
        joined = Pointcloud.from_join(*tuple(self.fine_calibrated_pointclouds))
        os.chdir(self.workdir)
        joined.save('cwipc_calibrate_calibrated.ply')
        self.ui.show_points('Inspect fine calibration result', joined)
        
    def skip_fine(self):
        for i in range(len(self.cameraserial)):
            self.fine_matrix.append([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ])
        self.fine_calibrated_pointclouds = self.coarse_calibrated_pointclouds
        
    def save(self):
        # Open3D Visualiser changes directory (!!?!), so change it back
        os.chdir(self.workdir)
        self.writeconfig()
        self.cleanup()
        
    def cleanup(self):
        self.pointclouds = []
        self.coarse_calibrated_pointclouds = []
        self.fine_calibrated_pointclouds = []
        self.refpointcloud = None
        self.grabber = None
        
    def get_pointclouds(self):
        # Create the canonical pointcloud, which determines the eventual coordinate system
        self.refpointcloud = Pointcloud.from_points(self.refpoints)
        # Get the number of cameras and their tile numbers
        maxtile = self.grabber.getcount()
        if DEBUG: print('maxtile', maxtile)
        # Grab one combined pointcloud and split it into tiles
        for i in range(10):
            pc = self.grabber.getpointcloud()
            if DEBUG: pc.save('cwipc_calibrate_captured.ply')
            self.pointclouds = pc.split()
            if len(self.pointclouds) == maxtile: break
            self.ui.show_error(f'Warning: got {len(self.pointclouds)} pointclouds in stead of {maxtile}. Retry.')
        assert len(self.pointclouds) == maxtile
        # xxxjack
        if DEBUG:
            joined = Pointcloud.from_join(self.pointclouds)
            joined.save('cwipc_calibrate_uncalibrated.ply')
        

    def align_pair(self, source, picked_id_source, target, picked_id_target, extended=False):
        assert(len(picked_id_source)>=3 and len(picked_id_target)>=3)
        assert(len(picked_id_source) == len(picked_id_target))
        corr = np.zeros((len(picked_id_source),2))
        corr[:,0] = picked_id_source
        corr[:,1] = picked_id_target

        p2p = open3d.registration.TransformationEstimationPointToPoint()
        trans_init = p2p.compute_transformation(source.get_o3d(), target.get_o3d(),
                 open3d.utility.Vector2iVector(corr))
        
        if not extended:
            return trans_init

        threshold = 0.01 # 3cm distance threshold
        reg_p2p = open3d.registration.registration_icp(source.get_o3d(), target.get_o3d(), threshold, trans_init,
                open3d.registration.TransformationEstimationPointToPoint())
        
        return reg_p2p.transformation
    
    def align_fine(self, source, target, threshold):
        trans_init = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
        reg_p2p = open3d.registration.registration_icp(source.get_o3d(), target.get_o3d(), threshold, trans_init,
                open3d.registration.TransformationEstimationPointToPoint())
        
        return reg_p2p.transformation

    def writeconfig(self):
        allcaminfo = ""
        for i in range(len(self.cameraserial)):
            serial = self.cameraserial[i]
            # Find matrix by applying what we found after the matrix read from the original config file (or the identity matrix)
            matrix = self.grabber.getmatrix(i)
            npMatrix = np.matrix(self.fine_matrix[i]) @ np.matrix(self.coarse_matrix[i]) @ np.matrix(matrix)
            matrix = npMatrix.tolist()
            matrixinfo = self.to_conf(matrix)
            caminfo = CONFIGCAMERA.format(serial=serial, matrixinfo=matrixinfo)
            allcaminfo += caminfo
        if self.near or self.far:
            distance = f'threshold_near="{self.near}" threshold_far="{self.far}"'
        else:
            distance = ''
        fileinfo = CONFIGFILE.format(cameras=allcaminfo, distance=distance)
        with open('cameraconfig.xml', 'w') as fp:
            fp.write(fileinfo)
 
    def to_conf(self, trans):
        s = "<values "
        for i in range(4):
            for j in range(4):
                s += f'v{i}{j}="{trans[i][j]}" '
                
        s += " />"
        return s   

  
