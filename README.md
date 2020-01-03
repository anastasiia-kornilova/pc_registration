# Description

This repo is devoted for building 3D scene from sequence of PCDs (Point Clouds) from Velodyne (TODO: add links to spec).

## From .bag to .pcd

To obtain .pcd files from .bag use the next command

`rosrun pcl_ros bag_to_pcd <bag-file.bag> /velodyne/pointcloud2 <directory for pcds>`

## Requirements

* Open3D 0.9.0