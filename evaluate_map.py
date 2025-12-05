import os
import glob
import open3d as o3d
import pickle
import numpy as np

import argparse


def compare_maps():

    parser = argparse.ArgumentParser()
    parser.add_argument("--ground-truth") #"maps/map_sequence_00_100_to_300_moving_vehicles.pcd"
    parser.add_argument("--map") #"maps/map_sequence_00_100_to_300_moving.pcd"
    parser.add_argument("--voxel-size", default=0.2, type=float)

    args = parser.parse_args()
    voxel_size_setting = args.voxel_size
    gt_pcd_path = args.ground_truth
    est_pcd_path = args.map

    gt_pcd_name = os.path.basename(gt_pcd_path)
    gt_pcd_dir = os.path.dirname(gt_pcd_path)
    naive_pcd_name = "naive_" + gt_pcd_name

    naive_pcd_path = os.path.join(gt_pcd_dir, naive_pcd_name)

    print("Evaluating precision and recall with voxel size=", voxel_size_setting)
    print("Evaluating map file at: ", est_pcd_path)
    print("Ground truth map:", gt_pcd_path)
    print("Naive map:", naive_pcd_path)
    # read pcd
    gt_pcd = o3d.io.read_point_cloud(gt_pcd_path)
    gt_pcd_voxelized = gt_pcd.voxel_down_sample(voxel_size=voxel_size_setting)


    naive_pcd = o3d.io.read_point_cloud(naive_pcd_path)
    naive_pcd_voxelized = naive_pcd.voxel_down_sample(voxel_size=voxel_size_setting)

    est_pcd = o3d.io.read_point_cloud(est_pcd_path)
    est_pcd_voxelized = est_pcd.voxel_down_sample(voxel_size=voxel_size_setting)

    gt_voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(gt_pcd_voxelized, voxel_size=voxel_size_setting)
    est_voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(est_pcd_voxelized, voxel_size=voxel_size_setting)
    naive_voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(naive_pcd_voxelized, voxel_size=voxel_size_setting)

    query1 = gt_voxel_grid.check_if_included(est_pcd_voxelized.points)
    query2 = est_voxel_grid.check_if_included(gt_pcd_voxelized.points)

    tp = np.sum(query1)
    fp = len(query1) - tp
    fn = len(query2) - np.sum(query2)
    print("tp= ",tp, "; fp=", fp, "; fn=", fn)

    precision = tp/(tp+fp)
    recall = tp / (tp + fn)
    print("precision=",precision, "; recall=", recall)

    print("===================================")
    print("Est voxel grid num voxels:",len(est_voxel_grid.get_voxels()))
    print("Est cloud num points:", len(est_pcd.points))

    print("Metrics using naive map")
    num_gt_voxels = len(gt_voxel_grid.get_voxels())
    num_gt_points = len(gt_pcd.points)
    print("GT voxel grid num voxels:",num_gt_voxels)
    print("GT cloud num points:",num_gt_points)

    num_naive_voxels = len(naive_voxel_grid.get_voxels())
    num_naive_points = len(naive_pcd.points)
    print("Naive voxel grid num voxels:",num_naive_voxels)
    print("Naive cloud num points:",num_naive_points)

    PR = tp / num_gt_voxels
    RR = 1 - fp/(num_naive_voxels - num_gt_voxels)
    F1 = 2*PR*RR/(PR+RR)
    print("PR=", PR, "; RR=", RR, "; F1=",F1)

if __name__ == "__main__":
    compare_maps()
