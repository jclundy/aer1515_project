import os
import glob
import open3d as o3d
import pickle
import numpy as np
import argparse

# kitti_data_default = "/media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/00/velodyne/"

Kitti_odometry_calib_file = "calib.txt"
Kitti_odometry_path = "/media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences"
Kitti_odometry_sequence = "00"
Kitti_odometry_image_folder = "image_2"

lidar_detections_file = "detections/lidar_detections.pkl"


def parse_args():
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument("--sequence", default=Kitti_odometry_sequence)
    parser.add_argument("--kitti_path", default=Kitti_odometry_path)
    parser.add_argument("--lidar_detections", default=lidar_detections_file)

    args = parser.parse_args()

    return args

def read_bin(bin_path):
    scan = np.fromfile(bin_path, dtype=np.float32)
    scan = scan.reshape((-1, 4))

    return scan

def pkl_visualization():
    args = parse_args()

    sequence = args.sequence
    kitti_path = args.kitti_path
    sequence_path = kitti_path + '/' + sequence + '/'
    pointcloud_data_path = sequence_path + "/velodyne/"
   
    lidar_result_path = sequence_path + '/' + args.lidar_detections

    print("Opening pickle files")
    with open(lidar_result_path,"rb") as fr:
        lidar_results = pickle.load(fr) # load pickle file

    # camera to lidar calibration
    calibration_file = sequence_path + '/' + Kitti_odometry_calib_file

    calib_data = np.loadtxt(calibration_file, delimiter=' ', dtype=str)
    transform_data = calib_data[4][1:13].astype(float).reshape((3,4))
    homogenous_transform = np.eye(4)
    homogenous_transform[0:3,0:4] = transform_data
    
   
    #points = np.fromfile(self.sample_file_list[index], dtype=np.float32).reshape(-1, 4)
    BIN_DIREC_PATH = os.path.join(sequence_path, "velodyne")
    bin_file_names_list = sorted(os.listdir(BIN_DIREC_PATH))

    for idx, bin_file_name in enumerate(bin_file_names_list):

        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.get_render_option().point_size = 1.0
        vis.get_render_option().background_color = np.zeros(3)

        print("idx: ", idx)
        bin_file_path = os.path.join(BIN_DIREC_PATH, bin_file_name)
        pointcloud = read_bin(bin_file_path)

        # read pcd
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pointcloud[:, :3])

        # pcd_path = "detections/000000.pcd" # TODO change path
        # pcd = o3d.io.read_point_cloud(pcd_path)
        # visualization_item.append(pcd)
        vis.add_geometry(pcd)

        # read pkl
        # pkl_path = "detections/000000.pkl" # TODO change path
        # with open(pkl_path, "rb") as fr:
        #     samples = pickle.load(fr)

        boxes_lidar = lidar_results[idx][0]["pred_boxes"]

        for box_3d in boxes_lidar:
            box_3d=box_3d.cpu().numpy()
            center = np.array([box_3d[0], box_3d[1], box_3d[2]])
            extent = np.array([box_3d[3], box_3d[4], box_3d[5]])
            theta = box_3d[6]
            rotation = np.array([[np.cos(theta), -np.sin(theta), 0.0],
                                [np.sin(theta), np.cos(theta), 0.0],
                                [0.0, 0.0, 1.0]])
            
            obb = o3d.geometry.OrientedBoundingBox(center=center, extent=extent, R=rotation)
            line_set = o3d.geometry.LineSet.create_from_oriented_bounding_box(obb)
            lines = np.asarray(line_set.lines)
            lines = np.concatenate([lines, np.array([[1, 4], [7, 6]])], axis=0)

            line_set.lines = o3d.utility.Vector2iVector(lines)
            color=(0, 1, 0)
            line_set.paint_uniform_color(color)
            # visualization_item.append(obb)
            vis.add_geometry(line_set)

        # vis.draw_geometries(visualization_item)
        vis.run()
        vis.destroy_window()    

if __name__ == "__main__":
    pkl_visualization()
