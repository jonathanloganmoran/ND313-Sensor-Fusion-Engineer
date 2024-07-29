/* ------------------------------------------------------------------------------
  * Project "1.1: LiDAR Obstacle Detection"
  * Authors     : Aaron Brown et al.
  *
  * Modified by : Jonathan Logan Moran (jonathan.moran107@gmail.com)
  *
  * Purpose of this file: Defines the Point Cloud Library (PCL) helper
  *         functions used to process, filter, segment, and separate PCD files.
  * ----------------------------------------------------------------------------
  */

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"


template<typename PointT>
class ProcessPointClouds {
public:
    // Constructor
    ProcessPointClouds();
    // Deconstructor
    ~ProcessPointClouds();

    // Prints the total number of points in the input `cloud`.
    void numPoints(
        typename pcl::PointCloud<PointT>::Ptr cloud
    );
    // Filters the point `cloud` to reduce the total number of data points. 
    typename pcl::PointCloud<PointT>::Ptr FilterCloud(
        typename pcl::PointCloud<PointT>::Ptr cloud, 
        float filterRes, 
        Eigen::Vector4f minPoint, 
        Eigen::Vector4f maxPoint
    );
    // Extracts the plane and obstacles using the Point Cloud Library (PCL).
    std::pair<
        typename pcl::PointCloud<PointT>::Ptr, 
        typename pcl::PointCloud<PointT>::Ptr
    > SeparateClouds(
        pcl::PointIndices::Ptr inliers, 
        typename pcl::PointCloud<PointT>::Ptr cloud
    );
    // Segments the input cloud into two using the Point Cloud Library (PCL).
    std::pair<
        typename pcl::PointCloud<PointT>::Ptr, 
        typename pcl::PointCloud<PointT>::Ptr
    > SegmentPlane(
        typename pcl::PointCloud<PointT>::Ptr cloud, 
        int maxIterations, 
        float distanceThreshold
    );
    // Performs Euclidean clustering with the Point Cloud Library (PCL).
    std::vector<
        typename pcl::PointCloud<PointT>::Ptr
    > Clustering(
        typename pcl::PointCloud<PointT>::Ptr cloud, 
        float clusterTolerance, 
        int minSize, 
        int maxSize
    );
    // Computes a 3D bounding box for the given point cloud `cluster`.
    Box BoundingBox(
        typename pcl::PointCloud<PointT>::Ptr cluster
    );
    // Saves the given `cloud` as an ASCII file to given `file` name.
    void savePcd(
        typename pcl::PointCloud<PointT>::Ptr cloud, 
        std::string file
    );
    // Loads the `.pcd` file from the given `file` name.
    typename pcl::PointCloud<PointT>::Ptr loadPcd(
        std::string file
    );
    // Returns a vector of file paths stored in the given `dataPath` folder.
    std::vector<
        boost::filesystem::path
    > streamPcd(
        std::string dataPath
    );
};
#endif /* PROCESSPOINTCLOUDS_H_ */