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
# OR
python mini_kitti_publisher.py --dir /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/08

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
# OR
python mini_kitti_publisher.py --dir /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/08


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
kiss_icp_pipeline --visualize <data-dir>

## Regular odometry
kiss_icp_pipeline --dataloader kitti --sequence 03 --visualize /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset
kiss_icp_pipeline --dataloader kitti --sequence 04 --visualize /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset
kiss_icp_pipeline --dataloader kitti --sequence 08 --visualize /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset
## Lidar-MOS cleaned scans
kiss_icp_pipeline --visualize /home/joseph/catkin/git/LiDAR-MOS/data/sequences/03/clean_scans
kiss_icp_pipeline --visualize /home/joseph/catkin/git/LiDAR-MOS/data/sequences/04/clean_scans
kiss_icp_pipeline --visualize /home/joseph/catkin/git/LiDAR-MOS/data/sequences/08/clean_scans
## Removert cleaned scans
kiss_icp_pipeline --visualize /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/removert/03_skip1/scan_static
kiss_icp_pipeline --visualize /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/removert/04_skip1/scan_static
kiss_icp_pipeline --visualize /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/removert/08_skip5/scan_static

# Removert
conda activate ros_env
source devel/setup.bash
roslaunch removert run_kitti.launch # if you use KITTI dataset 

# Lidar-MOS

# first update data_preparing.yaml file to point to right sequence
# run 
python utils/gen_residual_images.py
# then update post_processing.yaml 

# No more potentially dynamic objects
conda activate patchworkplus
python ground_segmentation.py


## openPCDet
conda activate openPCDet
cd ~/catkin/git/OpenPCDet/tools
## pv rcnn
python demo.py --cfg_file cfgs/kitti_models/pv_rcnn.yaml --ckpt ~/Downloads/pv_rcnn_8369.pth --data /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/03/velodyne/

## point pillars
python demo.py --cfg_file cfgs/kitti_models/pointpillar_copy.yaml --ckpt ~/Downloads/pointpillar_7728.pth --data /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/03/velodyne/

## voxelneXt
cd ~/catkin/git/VoxelNeXt/tools
python demo.py --cfg_file cfgs/kitti_models/voxelnext_copy.yaml --ckpt ../output/kitti_models/voxelnext/default/ckpt/checkpoint_epoch_16.pth --data /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/03/velodyne/


## save bounding boxes
python save_bounding_boxes.py --cfg_file cfgs/kitti_models/pv_rcnn.yaml --ckpt ~/Downloads/pv_rcnn_8369.pth --data /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/03/velodyne/ --output /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/object_detect_results/03/rcnn/
python save_bounding_boxes.py --cfg_file cfgs/kitti_models/pointpillar_copy.yaml --ckpt ~/Downloads/pointpillar_7728.pth --data /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/03/velodyne/ --output /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/object_detect_results/03/pointpillars/

python save_bounding_boxes.py --cfg_file cfgs/kitti_models/voxelnext_copy.yaml --ckpt ../output/kitti_models/voxelnext/default/ckpt/checkpoint_epoch_16.pth --data /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/03/velodyne/ --output /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/object_detect_results/03/voxelnext/

## ground projection
python ground_projection.py --data /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/03/ --boxes /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/object_detect_results/03/pointpillars/360/detections.pkl

# No More potentially dynamic objects
# Point Pillars 0.5
python save_bounding_boxes.py --cfg_file cfgs/kitti_models/pointpillar_copy.yaml --ckpt ~/Downloads/pointpillar_7728.pth --data /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/03/velodyne/ --output /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/NoMorePotentiallyDynamicObjects/04/pointpillars

#python save_bounding_boxes.py --cfg_file cfgs/kitti_models/pv_rcnn.yaml --ckpt ~/Downloads/pv_rcnn_8369.pth --data /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/03/velodyne/ --output /joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/NoMorePotentiallyDynamicObjects/03/pointpillars/0.5


# Ground projection - point pillars
python ground_projection.py --data /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/03/ --boxes /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/NoMorePotentiallyDynamicObjects/03/pointpillars/detections.pkl --conf 0.3
python ground_projection.py --data /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/03/ --boxes /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/NoMorePotentiallyDynamicObjects/03/pointpillars/detections.pkl --conf 0.5

python ground_projection.py --data /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/04/ --boxes /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/NoMorePotentiallyDynamicObjects/04/pointpillars/detections.pkl --conf 0.3
python ground_projection.py --data /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/04/ --boxes /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/NoMorePotentiallyDynamicObjects/04/pointpillars/detections.pkl --conf 0.5

# Ground projection - RCNN
python ground_projection.py --data /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/03/ --boxes /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/NoMorePotentiallyDynamicObjects/03/rcnn/detections.pkl --conf 0.3
python ground_projection.py --data /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/03/ --boxes /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/NoMorePotentiallyDynamicObjects/03/rcnn/detections.pkl --conf 0.5

python ground_projection.py --data /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/04/ --boxes /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/NoMorePotentiallyDynamicObjects/04/rcnn/detections.pkl --conf 0.3
python ground_projection.py --data /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/sequences/04/ --boxes /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/NoMorePotentiallyDynamicObjects/04/rcnn/detections.pkl --conf 0.5


# Map evaluation
## Map 03
### Point pillars
 python evaluate_map.py --ground-truth maps/map_sequence_03_0_to_801_moving_vehicles.pcd --map /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/NoMorePotentiallyDynamicObjects/03/pointpillars/0.3/map_sequence_03_0_to_801.pcd --voxel-size=0.2
 python evaluate_map.py --ground-truth maps/map_sequence_03_0_to_801_moving_vehicles.pcd --map /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/NoMorePotentiallyDynamicObjects/03/pointpillars/0.5/map_sequence_03_0_to_801.pcd --voxel-size=0.2
 
 ### RCNN
 python evaluate_map.py --ground-truth maps/map_sequence_03_0_to_801_moving_vehicles.pcd --map /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/NoMorePotentiallyDynamicObjects/03/rcnn/0.3/map_sequence_03_0_to_801.pcd --voxel-size=0.2
 python evaluate_map.py --ground-truth maps/map_sequence_03_0_to_801_moving_vehicles.pcd --map /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/NoMorePotentiallyDynamicObjects/03/rcnn/0.5/map_sequence_03_0_to_801.pcd --voxel-size=0.2
 
 ## Map 04
### Point pillars
 python evaluate_map.py --ground-truth maps/map_sequence_04_0_to_271_moving_vehicles.pcd --map /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/NoMorePotentiallyDynamicObjects/04/pointpillars/0.3/map_sequence_04_0_to_271.pcd --voxel-size=0.2
 python evaluate_map.py --ground-truth maps/map_sequence_04_0_to_271_moving_vehicles.pcd --map /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/NoMorePotentiallyDynamicObjects/04/pointpillars/0.5/map_sequence_04_0_to_271.pcd --voxel-size=0.2
 
 ### RCNN
 python evaluate_map.py --ground-truth maps/map_sequence_04_0_to_271_moving_vehicles.pcd --map /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/NoMorePotentiallyDynamicObjects/04/rcnn/0.3/map_sequence_04_0_to_271.pcd --voxel-size=0.2
 python evaluate_map.py --ground-truth maps/map_sequence_04_0_to_271_moving_vehicles.pcd --map /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset/results/NoMorePotentiallyDynamicObjects/04/rcnn/0.5/map_sequence_04_0_to_271.pcd --voxel-size=0.2

 # Lidar-MOS
## Data preparation
Copy over sequences 04 and 03 to LiDAR-MOS/data/sequences
## Generate semantic results
1. Modify config/data_preparing:
 For sequence 04
```
 # the folder of raw LiDAR scans
scan_folder: 'data/sequences/04/velodyne'
# ground truth poses file
pose_file: 'data/sequences/04/poses.txt'
# calibration file
calib_file: 'data/sequences/04/calib.txt'

# Outputs
# the suffix should be the same as num_last_n!
residual_image_folder: 'data/sequences/04/residual_images_1'
visualize: True
visualization_folder: 'data/sequences/04/visualization_1'
``

 For sequence 03
```
 # the folder of raw LiDAR scans
scan_folder: 'data/sequences/03/velodyne'
# ground truth poses file
pose_file: 'data/sequences/03/poses.txt'
# calibration file
calib_file: 'data/sequences/03/calib.txt'

# Outputs
# the suffix should be the same as num_last_n!
residual_image_folder: 'data/sequences/03/residual_images_1'
visualize: True
visualization_folder: 'data/sequences/03/visualization_1'
``
2. Run `python utils/gen_residual_images.py`.  Needs to be run separately for each data set 
## Generate MOS predictions
1. Update config file in the model_folder:
model_rangenet_residual_1/for_release/data_cfg.yaml
The split should be defined as:

```
split: # sequence numbers
  train:
    - 0
    - 1
    - 2
    - 5
    - 6
    - 7
    - 8
    - 9
    - 10
  valid:
    - 4
    - 3
  test:
    - 11
    - 12
    - 13
    - 14
    - 15
    - 16
    - 17
    - 18
    - 19
    - 20
    - 21
```

2. Run infer.py
This only needs to be run once.  It will do the segmentation for all the folders listed under valid in the model's data_cfg.yaml file
```
cd LiDAR-MOS/mos_RangeNet/tasks/semantic
python infer.py -d /home/joseph/catkin/git/LiDAR-MOS/data/ -l /home/joseph/catkin/git/LiDAR-MOS/logs/ -m /home/joseph/Downloads/model_rangenet_residual_1/for_release --split valid
```

## Clean scans
1. Ensure config/post-processing.yaml has the correct fields:
```
# the root of raw LiDAR scans
scan_root: 'data'
# the root of mos predictions
mos_pred_root: 'logs'

# Outputs
split: valid  # choose from (train, valid, test)
clean_scan_root: 'data'
```
2. Run utils/scan_cleaner.py
```
python utils/scan_cleaner.py
```
This will output the cleaned scans to /data/sequences/[SEQ]/clean_scans/ 

# LidarMOS

# python train.py -d /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset -ac /home/joseph/Downloads/model_rangenet_residual_1/custom_train/arch_cfg.yaml -dc /home/joseph/Downloads/model_rangenet_residual_1/custom_train/data_cfg.yaml -l /home/joseph/Downloads/model_rangenet_residual_1/custom_train/results/
python train.py -d /media/joseph/7E408025407FE1F7/Datasets/Kitti/odometry/dataset -ac /home/joseph/Downloads/model_rangenet_residual_1/custom_train/arch_cfg.yaml -l /home/joseph/catkin/git/LiDAR-MOS/training/ -dc /home/joseph/Downloads/model_rangenet_residual_1/custom_train/data_cfg.yaml
