import sys
import os
import numpy as np
import open3d
import pprint
import math

from .pointcloud import Pointcloud
from .cameraconfig import CameraConfig
from .ui import UI
DEBUG=False

FRONTAL_MATRIX = [
    [-0.9913142171561211, -0.06579945493198505, 0.11387078025024935, -0.011145954772718299],
    [0.06075368651814084, -0.9970346673683885, -0.04723199805282863,  1.0583022566736195],
    [0.11664095523701087, -0.03990368148755935, 0.9923722002178104,  -0.6241440078503737],
    [0, 0, 0, 1]
]    
class Calibrator:
    def __init__(self, distance, refpoints):
        self.ui = UI()
        self.cameraserial = []
        self.cameraconfig = None
        self.near = 0
        self.far = 0
        if distance:
            self.near = 0.5 * distance
            self.far = 2.0 * distance
        self.height_min = 0
        self.height_max = 0
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
        if clean:
            if os.path.exists('cameraconfig.xml'):
                os.unlink('cameraconfig.xml')
        elif reuse:
            pass
        elif os.path.exists('cameraconfig.xml'):
            self.ui.show_error('%s: cameraconfig.xml already exists, please supply --clean or --reuse argument' % sys.argv[0])
            sys.exit(1)
        self.grabber = grabber
        self.grabber.open()
        self.cameraserial = self.grabber.getserials()
        self.cameraconfig = CameraConfig('cameraconfig.xml', read=False)
        self.cameraconfig.copyFrom(self.grabber.cameraconfig)

    def issynthetic(self):
        return self.grabber.getserials()[0] == "synthetic"
        
    def auto(self):
        self.skip_coarse()
        self.skip_fine()
        # If we have a single camera and no matrix we apply the frontal 1m matrix
        if len(self.cameraserial) == 0:
            pass
        elif len(self.cameraserial) == 1:
            if self.grabber.getmatrix(0) == [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]:
                print("* Single camera setup, assume frontal camera 1m away and 1.2m high")
                self.coarse_matrix[0] = FRONTAL_MATRIX
            # Otherwise presume the matrix has already been set
        else:
            if self.grabber.getmatrix(0) == [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]:
                # Uninitialized multi-camera setup. Cannot do automatically
                self.ui.show_error('%s: multi-camera setup, please run calibrator manually' % sys.argv[0])
                sys.exit(1)
        
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
            grab_ok = self.ui.show_question('Can you select the reference points on the alignment target from this pointcloud?', canretry=True)
            if not grab_ok:
                self.ui.show_message('* discarding 10 pointclouds')
                for i in range(10):
                    self.grabber.getpointcloud()
                self.ui.show_message('* Grabbing pointclouds again')
                self.pointclouds = []
                self.get_pointclouds()
        
        
    def run_coarse(self):
        self.ui.show_prompt('Pick reference points on alignment target reference', isedit=True)
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
        joined = Pointcloud.from_join(self.coarse_calibrated_pointclouds)
        os.chdir(self.workdir)
        joined.save('cwipc_rs2calibrate_coarse.ply')
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
        
    def apply_bbox(self, bbox):
        if bbox:
            # Apply bounding box to pointclouds
            for i in range(len(self.coarse_calibrated_pointclouds)):
                self.coarse_calibrated_pointclouds[i] = self.coarse_calibrated_pointclouds[i].bbox(bbox)
        joined = Pointcloud.from_join(self.coarse_calibrated_pointclouds)
        self.ui.show_prompt('Inspect pointcloud after applying bounding box')
        self.ui.show_points('Inspect bounding box result', joined)
    
    def run_fine(self, correspondence, inspect):
        for i in range(len(self.cameraserial)):
            self.fine_matrix.append([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ])
        if len(self.coarse_calibrated_pointclouds) <= 1:
            self.ui.show_message('* Skipping fine-grained calibration: only one camera')
            self.fine_calibrated_pointclouds = self.coarse_calibrated_pointclouds
            return
        # We will align everything to the first camera
        # Note: this is wrong. We should align the pointcloud with the smallest angle
        # to camera 0. Then we should align the next pointcloud either to 0 or to
        # the previous one. Repeat until done.
        refPointcloud = self.coarse_calibrated_pointclouds[0]
        # XXXShishir: Get yaw rotation angles from coarse calibration
        camPositions = []
        for i in range(0, len(self.coarse_calibrated_pointclouds)):
            matrix = self.grabber.getmatrix(i)
            npMatrix = np.matrix(self.coarse_matrix[i]) @ np.matrix(matrix)
            camVector = npMatrix @ np.array([0, 0, 0, 1])
            print(f'xxxjack shape {np.shape(camVector)}')
            camVector = np.array([camVector[0,0],camVector[0,1], camVector[0,2]])
            print(f'Camera position of {i} is {camVector}')
            camPositions.append(camVector)
#            angle = math.atan2(self.coarse_matrix[i][1][0], self.coarse_matrix[i][0][0])
#            rotationAngles.append(angle)
#            print(f'Rotation angle for {i} is {angle}')
        # xxxjack compute dot products 
        dotProducts = []
        for i in range(len(camPositions)):
            dotProduct = np.dot(camPositions[0], camPositions[i])
            print(f'Dot-product {i} is {dotProduct}')
        assert 0
        assert len(self.fine_calibrated_pointclouds) == 0
        self.fine_calibrated_pointclouds.append(refPointcloud)
        if inspect:
            print(f'Fine matrix for camera {self.cameraserial[0]} is identity matrix, by definition')
        for i in range(1, len(self.coarse_calibrated_pointclouds)):
            srcPointcloud =  self.coarse_calibrated_pointclouds[i]
            refInd = 0;
            #XXXShishir set reference point cloud based on yaw angles from coarse calibration
            if i > 1:
                minAngle = abs(math.atan2(math.sin(rotationAngles[i]-rotationAngles[0]), math.cos(rotationAngles[i]-rotationAngles[0])))
                for j in range(1,i):
                    if (abs(math.atan2(math.sin(rotationAngles[i]-rotationAngles[j]), math.cos(rotationAngles[i]-rotationAngles[j]))) < minAngle):
                        refInd = j
                        minAngle = abs(math.atan2(math.sin(rotationAngles[i]-rotationAngles[j]), math.cos(rotationAngles[i]-rotationAngles[j])))
            refPointcloud = self.coarse_calibrated_pointclouds[refInd]
            newMatrix = self.align_fine(refPointcloud,srcPointcloud, correspondence)
            newPointcloud = srcPointcloud.transform(newMatrix)
            #XXXShishir apply the transformation for the reference used to camera 1
            #Incase the first camera use the identity matrix will be used in the transform so newPointCloud stays the same
            newPointcloud = newPointcloud.transform(self.fine_matrix[refInd])
            self.fine_calibrated_pointclouds.append(newPointcloud)
            self.fine_matrix[i] = newMatrix
            if inspect:
                print(f'Fine matrix for camera {self.cameraserial[i]} is:')
                pprint.pprint(newMatrix)
                showPCref = refPointcloud.colored((255, 0, 0))
                showPCsrc = srcPointcloud.colored((0, 255, 0))
                showPCdst = newPointcloud.colored((0, 0, 255))
                joined = Pointcloud.from_join((showPCref, showPCsrc, showPCdst))
                self.ui.show_prompt(f"Inspect alignment of {self.cameraserial[i]} (before: green, after: blue) to reference {self.cameraserial[0]} (red) ")
                self.ui.show_points('Inspect alignment result', joined)
        self.ui.show_prompt('Inspect the resultant merged pointclouds of all cameras')
        joined = Pointcloud.from_join(self.fine_calibrated_pointclouds)
        os.chdir(self.workdir)
        joined.save('cwipc_rs2calibrate_calibrated.ply')
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
            self.pointclouds = pc.split()
            if len(self.pointclouds) == maxtile: break
            self.ui.show_error(f'Warning: got {len(self.pointclouds)} pointclouds in stead of {maxtile}. Retry.')
        if len(self.pointclouds) != maxtile:
            exit(1)
        #
        # Save captured pointcloud (for possible use later)
        #
        joined = Pointcloud.from_join(self.pointclouds)
        joined.save('cwipc_rs2scalibrate_captured.ply')
        

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
        reg_p2p = open3d.registration.registration_icp(target.get_o3d(), source.get_o3d(), threshold, trans_init,
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
            self.cameraconfig.setmatrix(i, matrix)
        if self.near or self.far or self.height_min or self.height_max:
            self.cameraconfig.setbounds(self.near, self.far, self.height_min, self.height_max)
        self.cameraconfig.save()
         
  
