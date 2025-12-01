import os 
import sys
import time 
import copy 

import numpy as np
from numpy import linalg as LA

import open3d as o3d

import yaml

import sys
sys.path.insert(0, "../semantic-kitti-api")  # add Folder_2 path to search list

from auxiliary.laserscan import LaserScan, SemLaserScan

def read_bin(bin_path):
    scan = np.fromfile(bin_path, dtype=np.float32)
    scan = scan.reshape((-1, 4))

    return scan


##########################
# User only consider this block
##########################
sequence = "00"
data_dir = "/media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/" + sequence + "/" # should end with / 
node_skip = 4

num_points_in_a_scan = 150000 # for reservation (save faster) // e.g., use 150000 for 128 ray lidars, 100000 for 64 ray lidars, 30000 for 16 ray lidars, if error occured, use the larger value.

is_live_vis = False # recommend to use false 
is_o3d_vis = True
intensity_color_max = 200

is_near_removal = True
thres_near_removal = 2 # meter (to remove platform-myself structure ghost points)

##########################


#
# scan_dir = data_dir + "Scans"
scan_dir = data_dir + "velodyne"
# scan_dir = "/media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset_projected_0/sequences/00/ground_segmentation/non_ground"

scan_files = os.listdir(scan_dir) 
scan_files.sort()

# scan_idx_range_to_stack = [0, len(scan_files)] # if you want a whole map, use [0, len(scan_files)]
scan_idx_range_to_stack = [100, 300] # if you want a whole map, use [0, len(scan_files)]

pose_file = "00_poses_kitti.txt"
# pose_dir = "/home/joseph/catkin/scaloam_ws/src/SC-A-LOAM/utils/python/results/latest/"
pose_dir = data_dir
pose_file = "poses.txt"

# Label file
label_dir = "/media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/data_odometry_labels/dataset/sequences/00/labels"
# /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/data_odometry_labels/dataset/sequences/00/labels/000000.label

default_config="../semantic-kitti-api/config/semantic-kitti.yaml"

# open config file
try:
    print("Opening config file %s" % default_config)
    CFG = yaml.safe_load(open(default_config, 'r'))
except Exception as e:
    print(e)
    print("Error opening yaml file.")
    quit()

color_dict = CFG["color_map"]
#######################################################################
filter_options = ["moving", "vehicles", "ground", "sidewalk"]
# filter_options = ["moving", "vehicles"]
# filter_option = "moving"

#######################################################################

poses = []
f = open(pose_dir+pose_file, 'r')
while True:
    line = f.readline()
    if not line: break
    pose_SE3 = np.asarray([float(i) for i in line.split()])
    pose_SE3 = np.vstack( (np.reshape(pose_SE3, (3, 4)), np.asarray([0,0,0,1])) )
    poses.append(pose_SE3)
f.close()

 # this is for KITTI 
ExtrinsicLiDARtoPoseBase = np.asarray([-1.857e-03, -9.999e-01, -8.039e-03, -4.784e-03,
       -6.481e-03,  8.0518e-03, -9.999e-01, -7.337e-02,
        9.999e-01, -1.805e-03, -6.496e-03, -3.339e-01,
        0.0,        0.0,        0.0,        1.0])
ExtrinsicLiDARtoPoseBase = np.reshape(ExtrinsicLiDARtoPoseBase,(4,4))

#
assert (scan_idx_range_to_stack[1] > scan_idx_range_to_stack[0])
print("Merging scans from", scan_idx_range_to_stack[0], "to", scan_idx_range_to_stack[1])


#
vis = o3d.visualization.Visualizer() 
vis.create_window('Map', visible = True) 
vis.get_render_option().point_size = 1.0
vis.get_render_option().background_color = np.zeros(3)

nodes_count = 0
pcd_combined_for_vis = o3d.geometry.PointCloud()

for node_idx in range(len(scan_files)):
    if(node_idx < scan_idx_range_to_stack[0] or node_idx >= scan_idx_range_to_stack[1]):
        continue

    nodes_count = nodes_count + 1
    if( nodes_count % node_skip != 0): 
        if(node_idx != scan_idx_range_to_stack[0]): # to ensure the vis init 
            continue

    print("read keyframe scan idx", node_idx)

    scan_pose = poses[node_idx]

    scan_path = os.path.join(scan_dir, scan_files[node_idx])


    ''' PART 1
    Open Point cloud and Labels into SemanticLaserScan object
    '''
    print("scan_path=",scan_path)

    scan = SemLaserScan(color_dict, project=True)
    scan.open_scan(scan_path)

    filename, file_extension = os.path.splitext(scan_files[node_idx])
    label_path = os.path.join(label_dir, filename + '.label')
    print("label_path=",label_path)
    scan.open_label(label_path)
    scan.colorize()

    scan_pcd = o3d.geometry.PointCloud()
    mask = None

    """
  0 : "unlabeled"
  1 : "outlier"
  10: "car"
  11: "bicycle"
  13: "bus"
  15: "motorcycle"
  16: "on-rails"
  18: "truck"
  20: "other-vehicle"
  30: "person"
  31: "bicyclist"
  32: "motorcyclist"
  40: "road"
  44: "parking"
  48: "sidewalk"
  49: "other-ground"
  50: "building"
  51: "fence"
  52: "other-structure"
  60: "lane-marking"
  70: "vegetation"
  71: "trunk"
  72: "terrain"
  80: "pole"
  81: "traffic-sign"
  99: "other-object"
  252: "moving-car"
  253: "moving-bicyclist"
  254: "moving-person"
  255: "moving-motorcyclist"
  256: "moving-on-rails"
  257: "moving-bus"
  258: "moving-truck"
  259: "moving-other-vehicle"
    """

    ''' PART 2
    Filter based on point label
    '''
    mask = scan.sem_label != 1
    if("moving" in filter_options):
        mask &= (scan.sem_label < 252)
    if("vehicles" in filter_options):
        m1 = scan.sem_label < 10
        m2 = scan.sem_label > 32
        m3 = scan.sem_label < 252
        mask = (m1 | (m2 & m3)) & mask
    if("ground" in filter_options):
        mask &= (scan.sem_label != 40) # road
        mask &= (scan.sem_label != 49) # other ground
        mask &= (scan.sem_label != 60) # lane markings
    if("sidewalk" in filter_options):
        mask &= (scan.sem_label != 48)

    scan_pcd.points = o3d.utility.Vector3dVector(scan.points[mask])
    scan_pcd.colors = o3d.utility.Vector3dVector(scan.sem_label_color[mask])

    scan_pcd_global = scan_pcd.transform(ExtrinsicLiDARtoPoseBase)
    scan_pcd_global = scan_pcd.transform(scan_pose) # global coord, note that this is not deepcopy

    ''' PART 3
    Voxel-based downsampling
    '''
    reduced_scan = scan_pcd_global.voxel_down_sample(voxel_size=0.1)
    pcd_combined_for_vis += reduced_scan # open3d pointcloud class append is fast
 

print("Final downsampling")
pcd_combined_for_vis = pcd_combined_for_vis.voxel_down_sample(voxel_size=0.1)

vis.add_geometry(pcd_combined_for_vis)

vis.run()
vis.destroy_window()

# save rgb colored points 
# map_name = data_dir + "map_" + str(scan_idx_range_to_stack[0]) + "_to_" + str(scan_idx_range_to_stack[1]) + ".pcd"
# o3d.io.write_point_cloud(map_name, pcd_combined_for_vis)
# print("the map is save (path:", map_name, ")")