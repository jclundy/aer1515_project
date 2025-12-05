import os
import glob
import open3d as o3d
import pickle
import numpy as np
import argparse





def visualization():

    parser = argparse.ArgumentParser()
    parser.add_argument("--est")  # "maps/map_sequence_00_100_to_300_moving_vehicles.pcd"
    parser.add_argument("--voxel-size", default=0.5,type=float)
    parser.add_argument("--gt")  # "maps/map_sequence_00_100_to_300_moving_vehicles.pcd"
    parser.add_argument("--save", action='store_true')

    args = parser.parse_args()
    est_pcd_path = args.est
    gt_pcd_path = args.gt
    voxel_setting = args.voxel_size


    vis = o3d.visualization.Visualizer() 
    vis.create_window('Map', visible = True) 
    vis.get_render_option().point_size = 1.0
    vis.get_render_option().background_color = np.zeros(3)

    
    fp_color = np.array([235, 64, 52])/255
    tp_color = np.array([52, 235, 86])/255
    fn_color = np.array([162, 52, 235])/255
    # read pcd
    est_pcd = o3d.io.read_point_cloud(est_pcd_path)
    gt_pcd = o3d.io.read_point_cloud(gt_pcd_path)

    est_pcd_sampled = est_pcd.voxel_down_sample(voxel_size=voxel_setting)
    gt_pcd_sampled = gt_pcd.voxel_down_sample(voxel_size=voxel_setting)

    gt_voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(gt_pcd_sampled, voxel_size=voxel_setting)
    est_voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(est_pcd_sampled, voxel_size=voxel_setting)

    query1 = gt_voxel_grid.check_if_included(est_pcd_sampled.points)
    query2 = est_voxel_grid.check_if_included(gt_pcd_sampled.points)

    gt_pcd_colors = np.ones((len(gt_pcd_sampled.points),3)) *fn_color.reshape((1,3))
    est_pcd_colors = np.ones((len(est_pcd_sampled.points),3)) *fp_color.reshape((1,3))

    gt_pcd_colors[query2] = tp_color
    est_pcd_colors[query1] = tp_color

    gt_pcd_sampled.colors = o3d.utility.Vector3dVector(gt_pcd_colors)
    est_pcd_sampled.colors = o3d.utility.Vector3dVector(est_pcd_colors)

    combined_points = np.vstack((np.asarray(gt_pcd_sampled.points),np.asarray(est_pcd_sampled.points)))
    combined_colors = np.vstack((np.asarray(gt_pcd_sampled.colors),np.asarray(est_pcd_sampled.colors)))
    pcd_voxel_final = o3d.geometry.PointCloud()
    pcd_voxel_final.points = o3d.utility.Vector3dVector(combined_points)
    pcd_voxel_final.colors = o3d.utility.Vector3dVector(combined_colors)

    voxel_visual = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd_voxel_final, voxel_size=voxel_setting)

    # print("showing point clouds")
    # vis.create_window('Map', visible = True) 
    # vis.get_render_option().point_size = 1.0
    # vis.get_render_option().background_color = np.zeros(3)
    # vis.add_geometry(est_pcd_sampled)
    # vis.add_geometry(gt_pcd_sampled)

    # vis.run()
    # vis.destroy_window()

    print("showing voxel grid")
    vis.create_window('Map', visible = True)
    vis.get_render_option().point_size = 1.0
    vis.get_render_option().background_color = np.zeros(3)
    vis.add_geometry(voxel_visual)
    vis.run()
    vis.destroy_window()

    if(args.save):
        est_pcd_name = os.path.basename(est_pcd_path)
        est_pcd_dir = os.path.dirname(est_pcd_path)
        compare_pcd_name = "compare_" + est_pcd_name

        compare_pcd_path = os.path.join(est_pcd_dir, compare_pcd_name)

        o3d.io.write_point_cloud(compare_pcd_path, pcd_voxel_final)
        print("comparison map is saved to ", compare_pcd_path)




if __name__ == "__main__":
    visualization()
