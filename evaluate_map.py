import os
import glob
import open3d as o3d
import pickle
import numpy as np


voxel_size_setting = 0.5

def compare_maps():
    # vis = o3d.visualization.Visualizer() 
    # vis.create_window('Map', visible = True) 
    # vis.get_render_option().point_size = 1.0
    # vis.get_render_option().background_color = np.zeros(3)

    print("Evaluating precision and recall with voxel size=", voxel_size_setting)
    
    # read pcd
    gt_pcd_path = "maps/map_sequence_00_100_to_300_moving_vehicles.pcd"
    gt_pcd = o3d.io.read_point_cloud(gt_pcd_path)
    gt_pcd_voxelized = est_pcd.voxel_down_sample(voxel_size=voxel_size_setting)

    est_pcd_path = "maps/map_sequence_00_100_to_300_moving.pcd"
    est_pcd = o3d.io.read_point_cloud(est_pcd_path)
    est_pcd_voxelized = est_pcd.voxel_down_sample(voxel_size=voxel_size_setting)

    gt_voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(gt_pcd_voxelized)
    est_voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(est_pcd_voxelized)

    query1 = gt_voxel_grid.check_if_included(est_pcd_voxelized.points)
    query2 = est_voxel_grid.check_if_included(gt_pcd_voxelized.points)

    tp = np.sum(query1)
    fp = len(query1) - tp
    fn = len(query2) - np.sum(query2)
    print("tp= ",tp, "; fp=", fp, "; fn=", fn)

    precision = tp/(tp+fp)
    recall = tp / (tp + fn)
    print("precision=",precision, "; recall=", recall)

    # vis.add_geometry(voxel_grid)
    # # vis.add_geometry(pcd)

    # vis.run()
    # vis.destroy_window()

if __name__ == "__main__":
    compare_maps()
