# Stereo Reconstruct

ROS Wrapper for Stereo Reconstruction, generate Depth Image and Point Cloud by left and right images

* [双目立体视觉原理](https://spatial-ai.co/stereo-vision-reconstruct.html)

-----

## Build

```sh
catkin_make -j4
```

## Run

```sh
roslaunch stereo_reconstruct stereo_reconstruct.launch \
  camera:=mynteye left:=left_rect right:=right_rect mm:=true\
  rviz:=true colormap:=true
```

## Save Pointcloud

```sh
rosrun pcl_ros pointcloud_to_pcd input:=/my_cloud _prefix:=/tmp/pcd/vel_
```

## ELAS
* [LIBELAS: Library for Efficient Large-scale Stereo Matching](http://www.cvlibs.net/software/libelas/)
* [elas_ros (ROS Wiki)](http://wiki.ros.org/elas_ros)
