
# Inputs:
# 1) path to point clouds (.bin) format
# example: /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/00/velodyne
# 2) path to odometry file
# example: /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/00/poses.txt
# 3) Lidar to Pose base transformation:

# From params_kitt.yaml of removert:
'''
  # @ Sequence's BaseToLidar info
  ExtrinsicLiDARtoPoseBase: # this is for the KITTI (in the removert example, using SuMa poses written in the camera coord)
      [-1.857e-03, -9.999e-01, -8.039e-03, -4.784e-03,
       -6.481e-03,  8.0518e-03, -9.999e-01, -7.337e-02,
        9.999e-01, -1.805e-03, -6.496e-03, -3.339e-01,
        0.0,        0.0,        0.0,        1.0]
  # @ If you use the lidar-itself odometry (e.g., from A-LOAM or using odometry saver https://github.com/SMRT-AIST/interactive_slam/wiki/Example2), 
  #   use the below identity matrix, not the above KITTI pose setting (w.r.t the camera).
  # ExtrinsicLiDARtoPoseBase: [1.0, 0.0, 0.0, 0.0, 
  #                            0.0, 1.0, 0.0, 0.0, 
  #                            0.0, 0.0, 1.0, 0.0, 
  #                            0.0, 0.0, 0.0, 1.0]
'''