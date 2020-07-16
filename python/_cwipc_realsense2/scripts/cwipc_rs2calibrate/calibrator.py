import sys
import os
import numpy as np
import open3d

from .pointcloud import Pointcloud
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


def prompt(msg, isedit=False):
    stars = '*'*(len(msg)+2)
    print(stars)
    print('* ' + msg)
    print()
    print('- Inspect the pointcloud (use drag and mousehweel)')
    print('- Use +/= or -/_ to change point size')
    print('- press q or ESC when done')
    if isedit:
        print('- Select points with shift-leftclick, Deselect points with shift-rightclick')
        print('- Shift +/= or Shift -/_ to change selection indicator size')
        print('- ignore selection indicator colors, only the order is important')
    sys.stdout.flush()
        
def ask(msg, canretry=False):
    answered = False
    while not answered:
        print('* ', msg)
        if canretry:
            print('* Press y if it is fine, n to retry, or control-C to abort')
        else:
            print('* Press y if it is fine, or control-C to abort')
        print('? ', end='')
        sys.stdout.flush()
        answer = sys.stdin.readline().strip().lower()
        ok = answer == 'y'
        answered = ok
        if canretry and answer == 'n':
            answered = True
    return ok
                    
class Calibrator:
    def __init__(self, distance, refpoints):
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
        self.winpos = 100
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
            print('%s: cameraconfig.xml already exists, please supply --clean or --reuse argument' % sys.argv[0])
            sys.exit(1)
        # Set initial config file, for filtering parameters
        self.writeconfig()
        self.grabber = grabber
        self.cameraserial = self.grabber.getserials()

    def grab(self, noinspect):
        if not self.cameraserial:
            print('* No realsense cameras found')
            return False
        print('* Grabbing pointclouds')
        self.get_pointclouds()
        if DEBUG:
            for i in range(len(self.pointclouds)):
                print('Saving pointcloud {} to file'.format(i))
                self.pointclouds[i].save('pc-%d.ply' % i)
        #
        # First show the pointclouds for visual inspection.
        #
        if noinspect: return
        grab_ok = False
        while not grab_ok:
            sys.stdout.flush()
            for i in range(len(self.pointclouds)):
                prompt(f'Showing grabbed pointcloud from camera {i} for visual inspection')
                self.show_points(f'Inspect grab from {self.cameraserial[i]}', self.pointclouds[i], from000=True)
            grab_ok = ask('Can you select the balls on the cross from this pointcloud?', canretry=True)
            if not grab_ok:
                print('* discarding 10 pointclouds')
                for i in range(10):
                    self.get_pointclouds()
                print('* Grabbing pointclouds again')
                self.pointclouds = []
                self.get_pointclouds()
        
        
    def run_coarse(self):
        prompt('Pick red, orange, yellow, blue points on reference image', isedit=True)
        #
        # Pick reference points
        #
        refpoints = self.pick_points('Pick points on reference', self.refpointcloud)
        
        #
        # Pick points in images
        #
        for i in range(len(self.pointclouds)):
            matrix_ok = False
            while not matrix_ok:
                prompt(f'Pick red, orange, yellow, blue points on camera {i} pointcloud', isedit=True)
                pc_refpoints = self.pick_points(f'Pick points on {self.cameraserial[i]}', self.pointclouds[i], from000=True)
                info = self.align_pair(self.pointclouds[i], pc_refpoints, self.refpointcloud, refpoints, False)
                prompt(f'Inspect resultant orientation of camera {i} pointcloud')
                new_pc = self.pointclouds[i].transform(info)
                self.show_points(f'Result from {self.cameraserial[i]}', new_pc)
                matrix_ok = ask('Does that look good?', canretry=True)
            assert len(self.coarse_matrix) == i
            assert len(self.coarse_calibrated_pointclouds) == i
            self.coarse_matrix.append(info)
            self.coarse_calibrated_pointclouds.append(new_pc)
        #
        # Show result
        #
        prompt('Inspect the resultant merged pointclouds of all cameras')
        joined = Pointcloud.from_join(*tuple(self.coarse_calibrated_pointclouds))
        os.chdir(self.workdir)
        joined.save('cwipc_calibrate_coarse.ply')
        self.show_points('Inspect manual calibration result', joined)
        
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
            print('* Skipping fine-grained calibration: only one camera')
            self.fine_calibrated_pointclouds = self.coarse_calibrated_pointclouds
            return
        joined = Pointcloud.from_join(*tuple(self.coarse_calibrated_pointclouds))
        prompt('Inspect pointcloud after applying bounding box, before fine-grained calibration')
        self.show_points('Inspect bounding box result', joined)
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
        prompt('Inspect the resultant merged pointclouds of all cameras')
        joined = Pointcloud.from_join(*tuple(self.fine_calibrated_pointclouds))
        os.chdir(self.workdir)
        joined.save('cwipc_calibrate_calibrated.ply')
        self.show_points('Inspect fine calibration result', joined)
        
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
            print(f'Warning: got {len(self.pointclouds)} pointclouds in stead of {maxtile}. Retry.')
        assert len(self.pointclouds) == maxtile
        # xxxjack
        if DEBUG:
            joined = Pointcloud.from_join(self.pointclouds)
            joined.save('cwipc_calibrate_uncalibrated.ply')
        
    def pick_points(self, title, pc, from000=False):
        vis = open3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name=title, width=960, height=540, left=self.winpos, top=self.winpos)
        self.winpos += 50
        vis.add_geometry(pc.get_o3d())
        if from000:
            viewControl = vis.get_view_control()
            viewControl.set_front([0, 0, -1])
            viewControl.set_lookat([0, 0, 1])
            viewControl.set_up([0, -1, 0])
        vis.run() # user picks points
        vis.destroy_window()
        return vis.get_picked_points()

    def show_points(self, title, pc, from000=False):
        vis = open3d.visualization.Visualizer()
        vis.create_window(window_name=title, width=960, height=540, left=self.winpos, top=self.winpos)
        self.winpos += 50
        vis.add_geometry(pc.get_o3d())
        # Draw 1 meter axes (x=red, y=green, z=blue)
        axes = open3d.geometry.LineSet()
        axes.points = open3d.utility.Vector3dVector([[0,0,0], [1,0,0], [0,1,0], [0,0,1]])
        axes.lines = open3d.utility.Vector2iVector([[0,1], [0,2], [0,3]])
        axes.colors = open3d.utility.Vector3dVector([[1,0,0], [0,1,0], [0,0,1]])
        vis.add_geometry(axes)
        if from000:
            viewControl = vis.get_view_control()
            viewControl.set_front([0, 0, -1])
            viewControl.set_lookat([0, 0, 1])
            viewControl.set_up([0, -1, 0])
        vis.run()
        vis.destroy_window()
        
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

  
