# Project 1.1: LiDAR Obstacle Detection
#### By Jonathan L. Moran (jonathan.moran107@gmail.com)

## Objectives
* Process raw LiDAR data with filtering, segmentation, and clustering techniques;
* Perform obstacle detection to identify objects in the driving scene;
* Implement the core functions of the detection pipeline in C++;
* Render the scene (i.e., point cloud data) and detections using the Point Cloud Library (PCL).

## Tasks
### Segmentation
* ✅ [`E1.2.2`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-04-24-Course-1-Lidar-Exercises-Part-2.ipynb): Use PCL to segment the 3D LiDAR point cloud into `ground` plane and `obstacles` (`ProcessPointClouds<PointT>::SegmentPlane()` function);
* ✅ [`E1.2.3`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-04-24-Course-1-Lidar-Exercises-Part-2.ipynb): Use PCL to split the 3D LiDAR point cloud into the `ground` and `obstacles` instances (`ProcessPointClouds<PointT>::SeparateClouds()` function);
* ✅ (Optional) [`E1.2.5` and `E1.2.7`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-04-24-Course-1-Lidar-Exercises-Part-2.ipynb): Write a 2D segmentation algorithm with [RANSAC](https://en.wikipedia.org/wiki/Random_sample_consensus) to find the `ground` plane instance (`ransac2d::Ransac()` and `ransac2d::RansacPlane()` functions);
* ✅ (Optional) [`E1.2.6` and `E1.2.8`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-04-24-Course-1-Lidar-Exercises-Part-2.ipynb): "Fit" the 2D RANSAC segmentation algorithm to the point cloud to estimate the ground plane (`ransac2d::main()`);

### Clustering
* ✅ [`E1.3.1`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-05-13-Course-1-Lidar-Exercises-Part-3.ipynb): Use PCL to cluster the 3D LiDAR points with the built-in Euclidean Clustering algorithm and K-D Tree implementation (`ProcessPointClouds<PointT>::Clustering()`);
* ✅ [`E1.3.1`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-05-13-Course-1-Lidar-Exercises-Part-3.ipynb): Set the desired clustering hyperparameters for use with the `ProcessPointClouds<PointT>::Clustering()` function (`clusterTolerance`, `minSize` and `maxSize`);
* ✅ (Optional) [`E1.3.3`-`E1.3.5`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-05-13-Course-1-Lidar-Exercises-Part-3.ipynb): Create a K-D Tree implementation in C++ using only standard library functions (`kdtree::KdTree`);
* ✅ (Optional) [`E1.3.3`-`E1.3.4`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-05-13-Course-1-Lidar-Exercises-Part-3.ipynb): Write the core functions of the custom K-D Tree using only C++ standard library (`KdTree::insert()`, `KdTree::search()`);
* ✅ (Optional) [`E1.3.5`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-05-13-Course-1-Lidar-Exercises-Part-3.ipynb): Implement a custom Euclidean Clustering algorithm in C++ using the custom K-D Tree (`cluster::euclideanCluster()` and helper function `cluster::cluster()`);

### Downsampling
* ✅ [`E1.4.1`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-07-06-Course-1-Lidar-Exercises-Part-4.ipynb): Implement the `ProcessPointClouds<PointT>::FilterCloud()` function with `pcl::VoxelGrid()` to perform Voxel-based filtering to "downsample" the input point cloud (i.e., to reduce the total number of points);
* ✅ (Optional) [`E1.4.2(a)`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-07-06-Course-1-Lidar-Exercises-Part-4.ipynb): Extend the `ProcessPointClouds<PointT>::FilterCloud()` function with `pcl::CropBox()` to perform Region-based filtering to "crop" the input point cloud to a desired dimension (to eliminate unwanted scene points);
* ✅ (Optional) [`E1.4.2(b)`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-07-06-Course-1-Lidar-Exercises-Part-4.ipynb): Extend the `ProcessPointClouds<PointT>::FilterCloud()` function with `pcl::CropBox()` to perform Region-based filtering to "remove" the area encompassing the roof of the ego-vehicle (to eliminate unwanted scene points);

### "Real-time" Detection
* ✅ [`E1.4.3`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-07-06-Course-1-Lidar-Exercises-Part-4.ipynb): Implement the argument-overloaded `environment::cityBlock()` function to perform the "complete" LiDAR scan processing pipeline (segmentation, clustering, downsampling, detection) across _multiple_ `.pcd` files in a "streaming" manner (i.e., process and visualise the point cloud scans sequentially from a specified folder);
* ✅ (Optional) [`E1.4.1`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-07-06-Course-1-Lidar-Exercises-Part-4.ipynb): Adjust the voxel-based downsampling hyperparameters (`filterRes`) to achieve "real-time" performance (i.e., increase `filterRes` value to "reduce" the voxel-grid resolution, effectively eliminating a larger number of points and theoretically reducing compute time);

### Visualisation
* ✅ [`E1.1.3`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-04-14-Course-1-Lidar-Exercises-Part-1.ipynb): Use PCL to render the LiDAR point cloud file(s) with the `environment::renderPointCloud()` function;
* [`E1.2.1`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-04-24-Course-1-Lidar-Exercises-Part-2.ipynb): Use PCL to "process" the LiDAR point data;
* ✅ [`E1.3.2`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-05-13-Course-1-Lidar-Exercises-Part-3.ipynb): Use PCL to visualise the detected "clusters";
* ✅ [`E1.3.6`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-05-13-Course-1-Lidar-Exercises-Part-3.ipynb): Use PCL to estimate cluster bounding boxes (`ProcessPointClouds<PointT>::BoundingBox()`) and visualise them;
* ✅ (Optional) [`E1.2.4`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-04-24-Course-1-Lidar-Exercises-Part-2.ipynb): Use PCL to render the segmented `ground` plane and `obstacles` cloud with unique colours; 
* ✅ (Optional) [`E1.4.2(a)`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-07-06-Course-1-Lidar-Exercises-Part-4.ipynb): Adjust the region-based filtering hyperparameters (`minPoint`, `maxPoint`) to reduce point cloud "size" (i.e., "crop" the point cloud to a 'smaller' region by changing `minPoint`, `maxPoint` values).



## 1. Introduction


## 2. Programming Task
### 2.1. LiDAR Obstacle Detection

#### Background


#### Results


#### Prerequisites
In order to make use of this project, you must have the following dependencies installed —

C++:
* [C++11](https://en.wikipedia.org/wiki/C%2B%2B11);
* [Point Cloud Library (PCL) — v1.7.2](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.7.2);
* [Eigen — v3.2.92](https://eigen.tuxfamily.org/dox-3.2/);
* [gcc — v5.5](https://gcc.gnu.org/onlinedocs/gcc-5.5.0/gcc/).

These packages come pre-installed on the Udacity VM. **Note** The [`CMakeLists.txt`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/1-1-Lidar-Obstacle-Detection/CMakeLists.txt) file provided in this repo can be used locally if you have the same package versions as mentioned above. If you want to run this project locally (outside the Udacity workspace), please follow the steps under the [**Local Installation**](https://github.com/udacity/SFND_Lidar_Obstacle_Detection/tree/master#local-installation) section in the Udacity starter code `README` file.

#### Running and coompiling the programme
##### Setting the hyperparameters

##### Creating the executable
In order to create the executable for this project, we use `CMAKE`. To build the project, run the following console commands from inside the _root_ directory of the project:

```console
root@foobar:/../1-1-Lidar-Obstacle-Detection/#  mkdir build && cd build
root@foobar:/../1-1-Lidar-Obstacle-Detection/build/#  cmake ..
root@foobar:/../1-1-Lidar-Obstacle-Detection/build/#  make
```

##### Executing the programme
To run the programme, first verify that it has compiled successfully (you can check this by observing the `./environment` executable present inside the `"../build/"` sub-directory). Simply "launch" the programme executable from the console with the following command:

```console
root@foobar:/../1-1-Lidar-Obstacle-Detection/build/#  ./environment
```

This should open a new window (a PCL Viewer canvas instance) and display the following output:



##### Evaluating the results


#### Discussion


## 3. Closing Remarks
##### Alternatives


##### Extensions of task


## 4. Future Work
* ⬜️


## Credits
This assignment was prepared by Aaron Brown and Michael Maile of Mercedes-Benz Research & Development of North America (MBRDNA), 2021 (link [here](https://learn.udacity.com/nanodegrees/nd313/)).


References
* [] TBD.


Helpful resources:
* [`SFND_Lidar_Obstacle_Detection` | Starter code by @Udacity](https://github.com/udacity/SFND_Lidar_Obstacle_Detection);