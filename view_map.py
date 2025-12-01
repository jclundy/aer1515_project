import os
import glob
import open3d as o3d
import pickle
import numpy as np


def visualization():
    vis = o3d.visualization.Visualizer() 
    vis.create_window('Map', visible = True) 
    vis.get_render_option().point_size = 1.0
    vis.get_render_option().background_color = np.zeros(3)

    
    # read pcd
    pcd_path = "maps/map_sequence_00_100_to_300.pcd"
    pcd = o3d.io.read_point_cloud(pcd_path)

    vis.add_geometry(pcd)

    vis.run()
    vis.destroy_window()

if __name__ == "__main__":
    visualization()
