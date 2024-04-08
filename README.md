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

<img src=pic/global_map.png align = "middle"  width="600" />
