# fast-lio2-map-based-localization
map-based localization.Modified from [fast-lio2](https://github.com/hku-mars/FAST_LIO) and [s-fast-lio](https://github.com/zlwang7/S-FAST_LIO)


# 1.Prerequisites
just to follow [s-fast-lio](https://github.com/zlwang7/S-FAST_LIO),I won't go into further detail here.

# 2.Build 
```
cd ~/catkin_ws/src
git clone https://github.com/xz00/fast-lio2-map-based-localization.git
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```

# 3.Relocalization
## 3.1 build a global map
Recommend [fast-lio-sam](https://github.com/kahowang/FAST_LIO_SAM/tree/master) to build a global map.This work utilizes loop detection and point cloud matching, and updates the optimized pose to the front-end, enabling the creation of a high-quality map in a wide range of scenes. Can be used to create global maps based on map localization.
a large global map example:

<img src=pic/globalmap.png align = "middle"  width="600" />

## 3.2 modify the path of the global map
> config/velodyne_re.yaml
>> edit this param: globalmap_dir: "/home/xz/Desktop/catkin_fastlio_slam/data/totalmap.pcd"  # starts and ends with "/". 

## 3.3 run the launch and bag
```
roslaunch sfast_lio mapping_velodyne_relocalization.launch
rosbag play your-bag.bag
```

## 3.3 initialization
The initialization method is the same as [LIO-SAM_based_relocalization](https://github.com/Gaochao-hit/LIO-SAM_based_relocalization),complete initialization matching of point clouds by specifying an initialization position and direction in rviz.

## 3.4 start to relocalization
> note:this code can provide two kind of odometry.
> 1. "/Odometry_relocal",**this is just local scan register to local map**,so when IMU is not very stable and may drift,this odometry may stable.
> 2. "/Odometry",this is the result of loose coupling between IMU recursive results and point cloud matching results.

# 4.The principle of relocalization algorithm
The principle is very easy.After the pose is initialized, based on the forward propagation of IMU, when the point cloud is generated, it is registered with the local map, and the results of IMU and point cloud registration are updated through Bayesian rule. You can change some local map's params in the config/velodyne_re.yaml.

# 5.to do
5.1 upload the pcd and bag for reader to run an example.   [example PCD and bag](链接:https://pan.baidu.com/s/1No9ZvqtsM8LAxMmpLzKf0Q?pwd=nb73 提取码:nb73 复制这段内容后打开百度网盘手机App，操作更方便哦)

5.2 record a video to show this code how to use.

# 6.Acknowledgements
[fast-lio2](https://github.com/hku-mars/FAST_LIO) 

[s-fast-lio](https://github.com/zlwang7/S-FAST_LIO)

[fast-lio-sam](https://github.com/kahowang/FAST_LIO_SAM/tree/master)

[LIO-SAM_based_relocalization](https://github.com/Gaochao-hit/LIO-SAM_based_relocalization)
