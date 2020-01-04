# Description

This repo is devoted for building 3D scene from sequence of PCDs (Point Clouds) from Velodyne [VLP-16 Puck LITE](https://velodynelidar.com/products/puck-lite/).

Files are recorded using `rosbag` (TODO: maybe other effective approaches can be in use), then recorded bag-files are transformed to pcd-files (details are in section "From .bag to .pcd").

TODO: describe here Velodyne PCD format and thoughts on how it can be used.

## From .bag to .pcd

To obtain .pcd files from .bag use:

`rosrun pcl_ros bag_to_pcd <bag-file.bag> /velodyne/pointcloud2 <directory for pcds>`

The command above obtains PCD-files in binary format. To get ASCII format (can be used in understanding of Velodyne format) use:

`pcl_convert_pcd_ascii_binary <existing binary.pcd> <new ASCII.pcd> 0`

To convert all folder of PCDs to new folder use:

`for f in *.pcd ; do pcl_convert_pcd_ascii_binary $f ../new_pcds/$f 0 ; done`

## Requirements

* Open3d 0.9.0 (Version is IMPORTANT! Because library is raw enough and very different between versions.)
* Others from `requirements.txt`

## Problems

* Conversion to colored format
* Artifacts describing (TODO: search on approaches about "rolling shutter" for LIDARs)
* Last raw of scanner
* Approaches with colored ICP