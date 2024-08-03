# Project 1.1: LiDAR Obstacle Detection
#### By Jonathan L. Moran (jonathan.moran107@gmail.com)

## Objectives
* Process raw LiDAR data with filtering, segmentation, and clustering techniques;
* Perform obstacle detection to identify objects in the driving scene;
* Implement the core functions of the detection pipeline in C++;
* Render the scene (i.e., point cloud data) and detections using the Point Cloud Library (PCL).

## Tasks
### Segmentation
* ✅ Create a custom 3D `processPointClouds<PointT>::CustomSegmentPlane()` function to segment the 3D LiDAR point cloud of the _"Project 1.1" scene_ into `ground` plane and `obstacles` (see commit [`0156340`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/commit/015634019f0e9cc226171349c6969c66e9eb57b7) for reference);
* ✅ (Optional) [`E1.2.2`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-04-24-Course-1-Lidar-Exercises-Part-2.ipynb): Use PCL to segment the 3D LiDAR point cloud of the _"Simple Highway" scene_ into `ground` plane and `obstacles` (`ProcessPointClouds<PointT>::SegmentPlane()` function);
* ✅ (Optional) [`E1.2.3`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-04-24-Course-1-Lidar-Exercises-Part-2.ipynb): Use PCL to split the 3D LiDAR point cloud of the _"Simple Highway" scene_ into the `ground` and `obstacles` instances (`ProcessPointClouds<PointT>::SeparateClouds()` function);
* ✅ (Optional) [`E1.2.5` and `E1.2.7`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-04-24-Course-1-Lidar-Exercises-Part-2.ipynb): Write a 2D segmentation algorithm with [RANSAC](https://en.wikipedia.org/wiki/Random_sample_consensus) to find the `ground` plane instance (`ransac2d::Ransac()` and `ransac2d::RansacPlane()` functions) of the _"Ransac 2D" mock data_;
* ✅ (Optional) [`E1.2.6` and `E1.2.8`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-04-24-Course-1-Lidar-Exercises-Part-2.ipynb): "Fit" the 2D RANSAC segmentation algorithm to the point cloud to estimate the ground plane (`ransac2d::main()`) of the _"Ransac 2D" mock data_;

### Clustering
* ⬜️ Implement a custom 3D Euclidean Clustering algorithm in C++ using the custom 3D K-D Tree (`cluster::euclideanCluster3D()`) and its helper function (`cluster::cluster3D()`), then evaluate the results on _"3D K-D Tree" mock data_;
* ⬜️ Extend the custom 3D `cluster::euclideanCluster()` algorithm for use on the _"Project 1.1" scene_ (see commit [``]() for reference);
* ⬜️ Extend the custom 3D `kdtree::KdTree3D` implementation for use on the _"Project 1.1" scene_ (see commit [``]() for reference);
* ✅ [`E1.5.2`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/1-1/1-Lidar/1-1-Lidar-Obstacle-Detection/2024-07-27-Project-1-1-Lidar-Obstacle-Detection.ipynb): Write the core functions of the custom 3D K-D Tree (`kdtree::KdTree3D`) using only C++ standard library (`KdTree3D::insert()`, `KdTree3D::search()`) (see commit [`84c0eb1`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/commit/84c0eb1fa65752b474f4c280b450e93391f69b3d) for reference);
* ✅ [E1.5.2](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/1-1/1-Lidar/1-1-Lidar-Obstacle-Detection/2024-07-27-Project-1-1-Lidar-Obstacle-Detection.ipynb): Create a custom 3D K-D Tree implementation in C++ using only standard library functions (`kdtree::KdTree3D`) (see commit [`199d565`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/commit/199d5658be58d72ac4163293c37d3e881e225ffc) for reference);
* ✅ (Optional) [`E1.3.5`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-05-13-Course-1-Lidar-Exercises-Part-3.ipynb): Implement a custom 2D Euclidean Clustering algorithm in C++ using the custom 2D K-D Tree (`cluster::euclideanCluster()`) and its helper function (`cluster::cluster()`), then evaluate the results on _"2D KD-Tree" mock data_;
* ✅ (Optional) [`E1.3.3`-`E1.3.4`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-05-13-Course-1-Lidar-Exercises-Part-3.ipynb): Write the core functions of the custom 2D K-D Tree using only C++ standard library (`KdTree::insert()`, `KdTree::search()`), then evaluate the results on _"2D KD-Tree" mock data_;
* ✅ (Optional) [`E1.3.3`-`E1.3.5`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-05-13-Course-1-Lidar-Exercises-Part-3.ipynb): Create a custom 2D K-D Tree implementation in C++ using only standard library functions (`kdtree::KdTree`);
* ✅ (Optional) [`E1.3.1`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-05-13-Course-1-Lidar-Exercises-Part-3.ipynb): Set the desired clustering hyperparameters of the PCL `ProcessPointClouds<PointT>::Clustering()` function (`clusterTolerance`, `minSize` and `maxSize`) for use on the _"Simple Highway" scene_ ;
* ✅ (Optional) [`E1.3.1`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-05-13-Course-1-Lidar-Exercises-Part-3.ipynb): Use PCL to cluster the 3D LiDAR points of the _"Simple Highway" scene_ with the PCL Euclidean Clustering algorithm and the PCL K-D Tree implementation (`ProcessPointClouds<PointT>::Clustering()`);

### Downsampling
* ⬜️ Modify the input arguments to the `ProcessPointClouds<PointT>::FilterCloud()` function (i.e., `cloud`, `filterRes`) to perform Voxel-based filtering of the _"Project 1.1" scene_ using `pcl::VoxelGrid()` (see commit [``]() for reference);
* ⬜️ Modify the input arguments to the `ProcessPointClouds<PointT>::FilterCloud()` function (i.e., `minPoint`, `maxPoint`) to perform Region-based filtering of the _"Project 1.1" scene_ to "crop" the input cloud to desired dimensions using `pcl::CropBox()` (see commit [``]() for reference);
* ⬜️ Modify the variables defined in the `ProcessPointClouds<PointT>::FilterCloud()` function (i.e., the `Eigen::Vector4f(..)` values used in `roof.setMin(..)` and `roof.setMax(..)`) to perform Region-based filtering of the _"Project 1.1" scene_ to "remove" the area encompassing the roof of the ego-vehicle using `pcl::CropBox()` (see commit [``]() for reference);  
* ✅ (Optional) [`E1.4.1`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-07-06-Course-1-Lidar-Exercises-Part-4.ipynb): Implement the `ProcessPointClouds<PointT>::FilterCloud()` function with `pcl::VoxelGrid()` to perform Voxel-based filtering of the _"City Block" scene_ to "downsample" the input point cloud (i.e., to reduce the total number of points);
* ✅ (Optional) [`E1.4.2(a)`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-07-06-Course-1-Lidar-Exercises-Part-4.ipynb): Extend the `ProcessPointClouds<PointT>::FilterCloud()` function with `pcl::CropBox()` to perform Region-based filtering of the _"City Block" scene_ to "crop" the input point cloud to a desired dimension (to eliminate unwanted scene points);
* ✅ (Optional) [`E1.4.2(b)`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-07-06-Course-1-Lidar-Exercises-Part-4.ipynb): Extend the `ProcessPointClouds<PointT>::FilterCloud()` function with `pcl::CropBox()` to perform Region-based filtering of the _"City Block" scene_ to "remove" the area encompassing the roof of the ego-vehicle (to eliminate unwanted scene points);

### "Real-time" Detection
* ✅ `E1.5.0`: Create new `environment::projectPipeline()` function to perform "complete" LiDAR scan processing pipeline (segmentation, clustering, downsampling, detection) across _multiple_ `.pcd` files from the _"Project 1.1" scene_ in a "streaming" manner (see commit [`6d61846`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/commit/6d61846326305b4cf6d9497d2506e2db3dc6b042) for reference);
* ✅ (Optional) [`E1.4.3`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-07-06-Course-1-Lidar-Exercises-Part-4.ipynb): Implement the argument-overloaded `environment::cityBlock()` function to perform the "complete" LiDAR scan processing pipeline (segmentation, clustering, downsampling, detection) across _multiple_ `.pcd` files from the _"City Block" scene_ in a "streaming" manner (i.e., process and visualise the point cloud scans sequentially from a specified folder);
* ✅ (Optional) [`E1.4.1`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-07-06-Course-1-Lidar-Exercises-Part-4.ipynb): Adjust the voxel-based downsampling hyperparameters (`filterRes`) to achieve "real-time" performance on the _"City Block" scene_ (i.e., increase `filterRes` value to "reduce" the voxel-grid resolution, effectively eliminating a larger number of points and theoretically reducing compute time);

### Visualisation
* ✅ Use PCL to render the LiDAR point cloud file(s) of the _"Project 1.1" scene_ (see commit [`aeb8748`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/commit/aeb8748d96586dd876ec6837fe2b464916ed7e7b) for reference);
* ✅ Use PCL to render the "segmented" LiDAR point clouds generated by the custom segmentation function for the _"Project 1.1" scene_ (see commit [`41bac2b`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/commit/41bac2b9f666395633733700bf4c02c8b2b1dc0d) for reference);
* ⬜️ Use PCL to visualise the _"Project 1.1" scene_ clusters generated by the custom clustering function;
* ⬜️ Use PCL to estimate and visualise the estimated bounding boxes in the _"Project 1.1" scene_ (see commit [``]() for reference);
* ✅ (Optional) Use PCL to visualise the custom 3D K-D Tree core functions evaluated on the _"3D K-D Tree" mock data_ (see commit [`b845460`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/commit/b845460d795bcbc15597631ba883114c9a243b74) for reference);
* ✅ (Optional) [`E1.1.3`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-04-14-Course-1-Lidar-Exercises-Part-1.ipynb): Use PCL to render the LiDAR point cloud file of the _"Simple Highway" scene_ with the `environment::renderPointCloud()` function;
* ✅ (Optional) [`E1.2.1`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-04-24-Course-1-Lidar-Exercises-Part-2.ipynb): Use PCL to "process" the LiDAR point data in the _"Simple Highway" scene_;
* ✅ (Optional) [`E1.3.2`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-05-13-Course-1-Lidar-Exercises-Part-3.ipynb): Use PCL to visualise the detected "clusters" in the _"Simple Highway" scene_;
* ✅ (Optional) [`E1.3.6`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-05-13-Course-1-Lidar-Exercises-Part-3.ipynb): Use PCL to estimate the bounding boxes in the _"Simple Highway" scene_ with the `ProcessPointClouds<PointT>::BoundingBox()` function and visualise them;
* ✅ (Optional) [`E1.2.4`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-04-24-Course-1-Lidar-Exercises-Part-2.ipynb): Use PCL to render the segmented `ground` plane and `obstacles` cloud of the _"Simple Highway" scene_ with unique colours; 
* ✅ (Optional) [`E1.4.2(a)`](https://github.com/jonathanloganmoran/ND313-Sensor-Fusion-Engineer/blob/main/1-Lidar/Exercises/2024-07-06-Course-1-Lidar-Exercises-Part-4.ipynb): Adjust the region-based filtering hyperparameters (`minPoint`, `maxPoint`) to reduce point cloud "size" for the _"City Block" scene_ (i.e., "crop" the point cloud to a 'smaller' region by changing `minPoint`, `maxPoint` values).



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
* [_"2D KD-Tree" mock data_ | ]() — ;
* [_"Simple Highway" scene_ | ]();
* [_"Ransac 2D" mock data_ | ]();
* [_"City Block" scene_ | ]();
* [_"Project 1.1" scene_ | ]().