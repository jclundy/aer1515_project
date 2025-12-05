import os 
import copy 
import numpy as np
from numpy import linalg as LA
import open3d as o3d
import yaml
import argparse

def read_bin(bin_path):
    scan = np.fromfile(bin_path, dtype=np.float32)
    scan = scan.reshape((-1, 4))

    return scan

##########################
# Parse Arguments
##########################

parser = argparse.ArgumentParser()
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

if(MAP_CONFIG.get("sequence") is None):
    sequence = args.sequence
else:
    sequence = MAP_CONFIG["sequence"]

scan_dir = MAP_CONFIG["scans_path"]
if(MAP_CONFIG.get("removal_thresh") is None):
    thres_near_removal = None
else:
    thres_near_removal = MAP_CONFIG["removal_thresh"] # meter (to remove platform-myself structure ghost points)

if(MAP_CONFIG.get("skip") is None):
    node_skip = 1
else:
    node_skip = MAP_CONFIG["skip"]

calibration_path = MAP_CONFIG["calib_path"]
pose_file_path = MAP_CONFIG["pose_path"]
output_folder = MAP_CONFIG["output_folder"]
##########################

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
print("start_idx", startIdx)
print("end_idx", endIdx)
print("==========================================================")

scan_idx_range_to_stack = [startIdx, endIdx] # if you want a whole map, use [0, len(scan_files)]

#######################################################################


#######################################################################

poses = []
f = open(pose_file_path, 'r')
while True:
    line = f.readline()
    if not line: break
    pose_SE3 = np.asarray([float(i) for i in line.split()])
    pose_SE3 = np.vstack( (np.reshape(pose_SE3, (3, 4)), np.asarray([0,0,0,1])) )
    poses.append(pose_SE3)
f.close()


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
nodes_count = 0
pcd_combined = o3d.geometry.PointCloud()

for node_idx in range(len(scan_files)):
    if(node_idx < scan_idx_range_to_stack[0] or node_idx >= scan_idx_range_to_stack[1]):
        continue

    nodes_count = nodes_count + 1
    if( nodes_count % node_skip != 0): 
        if(node_idx != scan_idx_range_to_stack[0]): # to ensure the vis init 
            continue

    # print("read keyframe scan idx", node_idx)

    scan_pose = poses[node_idx]

    scan_path = os.path.join(scan_dir, scan_files[node_idx])

    scan_pcd = o3d.geometry.PointCloud()

    filename, file_extension = os.path.splitext(scan_path)
    if(file_extension == ".bin"):
        pointcloud_data = read_bin(scan_path)
        scan_pcd.points = o3d.utility.Vector3dVector(pointcloud_data[:, :3])
    else:
        scan_pcd = o3d.io.read_point_cloud(scan_path)

    scan_xyz_local = copy.deepcopy(np.asarray(scan_pcd.points))

    scan_pcd_global = scan_pcd.transform(ExtrinsicLiDARtoPoseBase)
    scan_pcd_global = scan_pcd.transform(scan_pose) # global coord, note that this is not deepcopy


    ''' PART 1
    Filter points too close
    '''
    if(thres_near_removal is not None):
        scan_ranges = LA.norm(scan_xyz_local, axis=1)
        eff_idxes = np.where (scan_ranges > thres_near_removal)
        scan_pcd_global = scan_pcd_global.select_by_index(eff_idxes[0])

        points_removed = len(scan_ranges) - len(eff_idxes[0])
        # print("number of points removed by range check:", points_removed)

    ''' PART 2
    Voxel-based downsampling
    '''
    reduced_scan = scan_pcd_global.voxel_down_sample(voxel_size=0.1)
    pcd_combined += reduced_scan # open3d pointcloud class append is fast

print("Final downsampling")
pcd_combined = pcd_combined.voxel_down_sample(voxel_size=0.1)

print("GT cloud num points: ", len(pcd_combined.points))

vis = o3d.visualization.Visualizer()
vis.create_window('Map', visible = True)
vis.get_render_option().point_size = 1.0
vis.get_render_option().background_color = np.zeros(3)

vis.add_geometry(pcd_combined)

vis.run()
vis.destroy_window()

if(args.visualize_only != True):
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    map_dir = output_folder
    map_name = "map_" + "sequence_" + sequence + "_" + str(scan_idx_range_to_stack[0]) + "_to_" + str(scan_idx_range_to_stack[1]) + ".pcd"
    map_path = os.path.join(map_dir, map_name)
    o3d.io.write_point_cloud(map_path, pcd_combined)
    print("the map is saved to:", map_path, ")")