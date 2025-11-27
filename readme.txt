# Conda
Cloning an environment
```
conda create --name myclone --clone myenv
```

Update path variable

conda env config vars set MY_VAR=[VALUE]

ie: conda env config vars set PATH=/usr/local/cuda-12/bin:/home/joseph/anaconda3/envs/openPCDet/bin:/home/joseph/anaconda3/condabin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin:/snap/bin

Local module install:
python -m pip install path/to/SomeProject

Set python version

conda install python=x.y

# FreeDOM
Running the algorithm:
```
conda activate ros_env
cd ~/catkin/FreeDOM_ws/
source devel/setup.bash
roslaunch freedom run_freedom.launch
```
In separate terminal:
```
conda activate ros_env
cd /media/joseph/7E408025407FE1F7/Datasets/KITTI-ros-bags/KITTI
# rosbag play [DATASET].bag
# for example:
rosbag play 00.bag
```
In a third terminal:

```
conda activate ros_env
cd ~/catkin/FreeDOM_ws/
# save static map
rostopic pub /save_map std_msgs/Empty "{}" -1
```

# generate ground truth
```
roslaunch freedom ground_truth_generate.launch
# in separate terminal
rosbag play 00.bag

```
# evaluate
```
roslaunch freedom static_map_evaluate.launch
```


# openPCDet
 1997  python demo.py --cfg_file cfgs/kitti_models/pointpillar.yaml --ckpt ~/Downloads/pointpillar_7728.pth --data_path "/media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/00/velodyne/"

 2008  python demo.py --cfg_file ../../VoxelNeXt/tools/cfgs/kitti_models/voxelnext.yaml --cpkt ~/Downloads/voxelnext_nuscenes_kernel1.pth --data_path "/media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/00/velodyne/"
 2009  python demo.py --cfg_file ../../VoxelNeXt/tools/cfgs/kitti_models/voxelnext.yaml --ckpt ~/Downloads/voxelnext_nuscenes_kernel1.pth --data_path "/media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/00/velodyne/"

 2062  python demo.py --cfg_file cfgs/kitti_models/voxelnext.yaml --ckpt ~/Downloads/voxelnext_nuscenes_kernel1.pth --data_path "/media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/00/velodyne/"

 2065  python demo.py --cfg_file cfgs/kitti_models/pointpillar.yaml --ckpt ~/Downloads/pointpillar_7728.pth --data_path "/media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/00/velodyne/"
 2066  python demo.py --cfg_file cfgs/kitti_models/pointpillar.yaml --ckpt ~/Downloads/pointpillar_7728.pth --data_path "/media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/00/velodyne/000010.bin"
 2067  history | grep "python demo.py"


# RVIz

rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map KITTI 5

# Conda and ROS packages
https://robostack.github.io/GettingStarted.html


# Kiss ICP
https://github.com/PRBonn/kiss-icp/blob/main/python/README.md
kiss_icp_pipeline --dataloader kitti --sequence 00 --visualize /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset