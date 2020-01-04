# Description

This repo is devoted for building 3D scene from sequence of PCDs (Point Clouds) from Velodyne (TODO: add links to spec).

## From .bag to .pcd

To obtain .pcd files from .bag use the next command:

`rosrun pcl_ros bag_to_pcd <bag-file.bag> /velodyne/pointcloud2 <directory for pcds>`

The command above obtains PCD-files in binary format. To get ASCII format use the next command:

`pcl_convert_pcd_ascii_binary <existing binary.pcd> <new ASCII.pcd> 0`

To convert all folder of PCDs to new folder use:

`for f in *.pcd ; do pcl_convert_pcd_ascii_binary $f ../new_pcds/$f 0 ; done`

## Requirements

* Open3D 0.9.0