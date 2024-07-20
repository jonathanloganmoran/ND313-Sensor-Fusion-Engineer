# Project 1.1: LiDAR Obstacle Detection
#### By Jonathan L. Moran (jonathan.moran107@gmail.com)

## Objectives


## Tasks


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