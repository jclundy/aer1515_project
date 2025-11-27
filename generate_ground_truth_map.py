import os 
import sys
import time 
import copy 

import numpy as np
from numpy import linalg as LA

import open3d as o3d

def read_bin(bin_path):
    scan = np.fromfile(bin_path, dtype=np.float32)
    scan = scan.reshape((-1, 4))

    return scan


##########################
# User only consider this block
##########################
sequence = "00"
data_dir = "/media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/" + sequence + "/" # should end with / 
node_skip = 2

num_points_in_a_scan = 150000 # for reservation (save faster) // e.g., use 150000 for 128 ray lidars, 100000 for 64 ray lidars, 30000 for 16 ray lidars, if error occured, use the larger value.

is_live_vis = False # recommend to use false 
is_o3d_vis = True
intensity_color_max = 200

is_near_removal = True
thres_near_removal = 2 # meter (to remove platform-myself structure ghost points)

##########################


#
# scan_dir = data_dir + "Scans"
# scan_dir = data_dir + "velodyne"
scan_dir = "/media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset_projected_0/sequences/00/ground_segmentation/non_ground"
scan_files = os.listdir(scan_dir) 
scan_files.sort()

# scan_idx_range_to_stack = [0, len(scan_files)] # if you want a whole map, use [0, len(scan_files)]
scan_idx_range_to_stack = [100, 150] # if you want a whole map, use [0, len(scan_files)]

# pose_file = "optimized_poses.txt"
pose_dir = "/home/joseph/catkin/scaloam_ws/src/SC-A-LOAM/utils/python/results/latest/"
# pose_file = "poses.txt"
# pose_file = "03_gt_tum.txt"

pose_file = "00_poses_kitti.txt"

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
pcd_combined_for_save = None

# The scans from 000000.pcd should be prepared if it is not used (because below code indexing is designed in a naive way)

# manually reserve memory for fast write  
num_all_points_expected = int(num_points_in_a_scan * np.round((scan_idx_range_to_stack[1] - scan_idx_range_to_stack[0])/node_skip))

np_xyz_all = np.empty([num_all_points_expected, 3])
np_intensity_all = np.empty([num_all_points_expected, 1])
curr_count = 0

for node_idx in range(len(scan_files)):
    if(node_idx < scan_idx_range_to_stack[0] or node_idx >= scan_idx_range_to_stack[1]):
        continue

    nodes_count = nodes_count + 1
    if( nodes_count % node_skip is not 0): 
        if(node_idx is not scan_idx_range_to_stack[0]): # to ensure the vis init 
            continue

    print("read keyframe scan idx", node_idx)

    scan_pose = poses[node_idx]

    scan_path = os.path.join(scan_dir, scan_files[node_idx])

    # scan_pcd = o3d.io.read_point_cloud(scan_path)
    scan_pcd = o3d.geometry.PointCloud()


    filename, file_extension = os.path.splitext(scan_path)
    if(file_extension == ".bin"):
        pointcloud_data = read_bin(scan_path)
        scan_pcd.points = o3d.utility.Vector3dVector(pointcloud_data[:, :3])
    else:
        scan_pcd = o3d.io.read_point_cloud(scan_path)

    scan_xyz_local = copy.deepcopy(np.asarray(scan_pcd.points))

 
    scan_intensity = np.ones((len(scan_pcd.points)))
    # scan_intensity_colors_idx = np.round( (color_table_len-1) * np.minimum( 1, np.maximum(0, scan_intensity / intensity_color_max) ) )
    # scan_intensity_colors = color_table[scan_intensity_colors_idx.astype(int)]

    scan_pcd_global = scan_pcd.transform(ExtrinsicLiDARtoPoseBase)
    scan_pcd_global = scan_pcd.transform(scan_pose) # global coord, note that this is not deepcopy
    # scan_pcd_global.colors = o3d.utility.Vector3dVector(scan_intensity_colors)
    scan_pcd_global.colors = o3d.utility.Vector3dVector(np.ones((len(scan_pcd.points), 3)))
    scan_xyz = np.asarray(scan_pcd_global.points)

    scan_intensity = np.expand_dims(scan_intensity, axis=1) 
    scan_ranges = LA.norm(scan_xyz_local, axis=1)

    if(is_near_removal):
        eff_idxes = np.where (scan_ranges > thres_near_removal)
        scan_xyz = scan_xyz[eff_idxes[0], :]
        scan_intensity = scan_intensity[eff_idxes[0], :]

        scan_pcd_global = scan_pcd_global.select_by_index(eff_idxes[0])

    if(is_o3d_vis):
        reduced_scan = scan_pcd_global.voxel_down_sample(voxel_size=0.1)
        pcd_combined_for_vis += reduced_scan # open3d pointcloud class append is fast 

    # save 
    np_xyz_all[curr_count:curr_count + scan_xyz.shape[0], :] = scan_xyz
    np_intensity_all[curr_count:curr_count + scan_xyz.shape[0], :] = scan_intensity

    curr_count = curr_count + scan_xyz.shape[0]
    print(curr_count)
 

print("Final downsampling")
pcd_combined_for_vis = pcd_combined_for_vis.voxel_down_sample(voxel_size=0.1)

vis.add_geometry(pcd_combined_for_vis)

vis.run()
vis.destroy_window()

# save rgb colored points 
# map_name = data_dir + "map_" + str(scan_idx_range_to_stack[0]) + "_to_" + str(scan_idx_range_to_stack[1]) + ".pcd"
# o3d.io.write_point_cloud(map_name, pcd_combined_for_vis)
# print("the map is save (path:", map_name, ")")

'''
clouds_list = [np.load(cloud_file).reshape(-1, args.point_dims) for cloud_file in args.clouds_file_list]


input_pkl_path = os.path.join(DATASET_DIREC_PATH, args.bbox_file_name + ".pkl") # write input pkl path
start_idx = 0

ground_heights_path = os.path.join(DATASET_DIREC_PATH, "ground_heights.txt")

with open(input_pkl_path,"rb") as fr:
    results = pickle.load(fr) # load pickle file

'''