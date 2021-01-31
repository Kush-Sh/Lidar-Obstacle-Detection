# Lidar Obstacle Detection

<img src="https://github.com/Kush-Sh/Lidar-Obstacle-Detection/blob/main/pcdstreamdetection.gif" width="948" height="532" />

# Real World Point Cloud
<img src="https://github.com/Kush-Sh/Lidar-Obstacle-Detection/blob/main/Point%20cloud.jpg" width="948" height="532" />

### Welcome to the lidar obstacle detection project for self-driving cars.

In this project we will be talking about lidar point cloud data, which is the process of taking data from lidar and using voxel grid filtering, RANSAC, KD-Tree, and Euclidean clustering algorithms for a better understanding of the world around us.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.


## Installation

### Linux Ubuntu 16

Install PCL, C++

The link here is very helpful, 
https://larrylisky.com/2014/03/03/installing-pcl-on-ubuntu/

A few updates to the instructions above were needed.

* libvtk needed to be updated to libvtk6-dev instead of (libvtk5-dev). The linker was having trouble locating libvtk5-dev while building, but this might not be a problem for everyone.

* BUILD_visualization needed to be manually turned on, this link shows you how to do that,
https://pcl.readthedocs.io/projects/tutorials/en/latest/#visualization

