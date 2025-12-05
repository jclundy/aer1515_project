import os
import glob
import open3d as o3d
import pickle
import numpy as np
import argparse





def visualization():

    parser = argparse.ArgumentParser()
    parser.add_argument("--pcd")  # "maps/map_sequence_00_100_to_300_moving_vehicles.pcd"
    parser.add_argument("--voxel-size", default=0.5)

    args = parser.parse_args()
    voxel_size_setting = args.voxel_size
    pcd_path = args.pcd


    vis = o3d.visualization.Visualizer() 
    vis.create_window('Map', visible = True) 
    vis.get_render_option().point_size = 1.0
    vis.get_render_option().background_color = np.zeros(3)

    
    # read pcd
    pcd = o3d.io.read_point_cloud(pcd_path)


    vis.create_window('Map', visible = True)
    vis.get_render_option().point_size = 1.0
    vis.get_render_option().background_color = np.zeros(3)
    vis.add_geometry(pcd)

    vis.run()
    vis.destroy_window()

    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,
                                                            voxel_size=voxel_size_setting)
    vis.add_geometry(voxel_grid)
    vis.run()
    vis.destroy_window()




if __name__ == "__main__":
    visualization()
