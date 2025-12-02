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
# Parse Arguments
##########################
default_filter_options = ["moving", "vehicles"]


import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--sequence", default="00")
parser.add_argument("--config") #"configs/config0.yaml"
parser.add_argument("--visualize-only", action='store_true') #"configs/config0.yaml"

args = parser.parse_args()
map_config_file = args.config
# open config file
try:
    print("Opening map config file %s" % map_config_file)
    MAP_CONFIG = yaml.safe_load(open(map_config_file, 'r'))
except Exception as e:
    print(e)
    print("Error opening yaml file.")
    quit()

if(MAP_CONFIG.get("filters") is None):
    filter_options = []
else:
    filter_options = MAP_CONFIG["filters"]

##########################
# User only consider this block
##########################
if(MAP_CONFIG.get("sequence") is None):
    sequence = args.sequence
else:
    sequence = MAP_CONFIG["sequence"]

data_path = MAP_CONFIG["data_path"]
data_dir = os.path.join(data_path, sequence)

thres_near_removal = MAP_CONFIG["removal_thresh"] # meter (to remove platform-myself structure ghost points)

if(MAP_CONFIG.get("skip") is None):
    node_skip = 1
else:
    node_skip = MAP_CONFIG["skip"]

# filter_options = ["moving", "vehicles", "ground", "sidewalk"]
# filter_options = ["moving", "vehicles"]
# filter_options = ["moving"]


##########################

scan_dir = os.path.join(data_dir, "velodyne")

scan_files = os.listdir(scan_dir) 
scan_files.sort()

if(MAP_CONFIG.get("start_idx") is None):
    startIdx = 0
else:
    startIdx = MAP_CONFIG["start_idx"]

if(MAP_CONFIG.get("end_idx") is None):
    endIdx = len(scan_files)
else:
    endIdx = MAP_CONFIG["end_idx"]

print("==========================================================")
print("Running ground truth map generation with these parameters:")
print("sequence", sequence)
print("removal thresh", thres_near_removal)
print("skip", node_skip)
print("filter options", filter_options)
print("start_idx", startIdx)
print("end_idx", endIdx)
print("==========================================================")

scan_idx_range_to_stack = [startIdx, endIdx] # if you want a whole map, use [0, len(scan_files)]

# pose_file = "00_poses_kitti.txt"
# pose_dir = "/home/joseph/catkin/scaloam_ws/src/SC-A-LOAM/utils/python/results/latest/"
pose_dir = data_dir
pose_file = "poses.txt"

# Label file
label_dir = os.path.join(data_dir,"labels")

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


#######################################################################

poses = []
pose_file_path = os.path.join(pose_dir, pose_file)
f = open(pose_file_path, 'r')
while True:
    line = f.readline()
    if not line: break
    pose_SE3 = np.asarray([float(i) for i in line.split()])
    pose_SE3 = np.vstack( (np.reshape(pose_SE3, (3, 4)), np.asarray([0,0,0,1])) )
    poses.append(pose_SE3)
f.close()

calibration_path = os.path.join(data_dir, "calib.txt")

calib_data = np.loadtxt(calibration_path, delimiter=' ', dtype=str)
transform_data = calib_data[4][1:13].astype(float).reshape((3,4))
H_r4 = [0.0, 0.0, 0.0, 1.0]

ExtrinsicLiDARtoPoseBase = np.vstack([transform_data, H_r4])
print("ExtrinsicLiDARtoPoseBase.shape=", ExtrinsicLiDARtoPoseBase.shape)
print("ExtrinsicLiDARtoPoseBase=", ExtrinsicLiDARtoPoseBase)

#
assert (scan_idx_range_to_stack[1] > scan_idx_range_to_stack[0])
print("Merging scans from", scan_idx_range_to_stack[0], "to", scan_idx_range_to_stack[1])


#
vis = o3d.visualization.Visualizer() 
vis.create_window('Map', visible = True) 
vis.get_render_option().point_size = 1.0
vis.get_render_option().background_color = np.zeros(3)

nodes_count = 0
pcd_combined_gt = o3d.geometry.PointCloud()
pcd_combined_naive = o3d.geometry.PointCloud()

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
    scan_pcd_naive = o3d.geometry.PointCloud()
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
    ''' PART 2a
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

    scan_xyz_local = copy.deepcopy(np.asarray(scan_pcd.points))

    scan_pcd_global = scan_pcd.transform(ExtrinsicLiDARtoPoseBase)
    scan_pcd_global = scan_pcd.transform(scan_pose) # global coord, note that this is not deepcopy


    ''' PART 2b
    Filter points too close
    '''
    if(thres_near_removal is not None):
        scan_ranges = LA.norm(scan_xyz_local, axis=1)
        eff_idxes = np.where (scan_ranges > thres_near_removal)
        scan_pcd_global = scan_pcd_global.select_by_index(eff_idxes[0])

        points_removed = len(scan_ranges) - len(eff_idxes[0])
        print("number of points removed by range check:", points_removed)

    ''' PART 3
    Voxel-based downsampling
    '''
    reduced_scan = scan_pcd_global.voxel_down_sample(voxel_size=0.1)
    pcd_combined_gt += reduced_scan # open3d pointcloud class append is fast

    scan_pcd_naive.points = o3d.utility.Vector3dVector(scan.points)
    scan_pcd_naive.colors = o3d.utility.Vector3dVector(scan.sem_label_color)
    pcd_combined_naive += scan_pcd_naive.transform(ExtrinsicLiDARtoPoseBase).transform(scan_pose).voxel_down_sample(voxel_size=0.1)
 

print("Final downsampling")
pcd_combined_gt = pcd_combined_gt.voxel_down_sample(voxel_size=0.1)
pcd_combined_naive = pcd_combined_naive.voxel_down_sample(voxel_size=0.1)

print("GT cloud num points: ", len(pcd_combined_gt.points))
print("Naive cloud num points: ", len(pcd_combined_naive.points))

vis.add_geometry(pcd_combined_gt)

vis.run()
vis.destroy_window()

vis.create_window('Map', visible = True)
vis.get_render_option().point_size = 1.0
vis.get_render_option().background_color = np.zeros(3)
vis.add_geometry(pcd_combined_naive)

vis.run()
vis.destroy_window()

if(args.visualize_only != True):
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    map_dir = os.path.join(cur_dir, "maps")
    map_name = "map_" + "sequence_" + sequence + "_" + str(scan_idx_range_to_stack[0]) + "_to_" + str(scan_idx_range_to_stack[1]) + "_" + "_".join(filter_options) + ".pcd"
    map_path = os.path.join(map_dir, map_name)
    o3d.io.write_point_cloud(map_path, pcd_combined_gt)
    print("the map is save to:", map_path, ")")

    naive_map_name = "naive_" + map_name
    naive_path = os.path.join(map_dir, naive_map_name)
    o3d.io.write_point_cloud(naive_path, pcd_combined_naive)
    print("the naive acculmulated map is saved to ", naive_path)