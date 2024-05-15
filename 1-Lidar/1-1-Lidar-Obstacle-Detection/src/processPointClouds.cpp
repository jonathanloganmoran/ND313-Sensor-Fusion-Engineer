/* ----------------------------------------------------------------------------
  * Project "1.1: LiDAR Obstacle Detection"
  * Authors     : Aaron Brown et al.
  *
  * Modified by : Jonathan Logan Moran (jonathan.moran107@gmail.com)
  *
  * Purpose of this file: Implements the Point Cloud Library (PCL) helper
  *         functions used to process, filter, segment, and separate PCD files.
  * ---------------------------------------------------------------------------
  */

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


/** Extracts the plane and obstacles using the Point Cloud Library (PCL).
 * 
 * Extracts the ground plane from the input `cloud` given the estimated set
 * of `inliers`, i.e., the points determined to belong to the ground plane.
 * 
 * The extracted ground plane points are then copied into a new point cloud
 * instance (`ground`). The remaining points are copied into an `obstacles`
 * cloud. The two point clouds are returned in an `std::pair` instance, with
 * the `first` index containing the `ground` and the `second` containing the
 * `obstacles. 
 * 
 * @brief   Separates the point cloud into "ground plane" and "obstacles".
 * @param   inliers     Set of indices to extract into a new point cloud.
 * @param   cloud       Point cloud instance to "separate".
 * @returns segResult   The pair of point cloud instances.
 */
template<typename PointT> std::pair<
    typename pcl::PointCloud<PointT>::Ptr, 
    typename pcl::PointCloud<PointT>::Ptr
> ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers, 
    typename pcl::PointCloud<PointT>::Ptr cloud
) {
    /** E1.2.3: Separating the ground plane. **/
    //typename pcl::PointCloud<PointT>::Ptr obstacles = new pcl::PointCloud<PointT>();
    typename pcl::PointCloud<PointT>::Ptr obstacles(
        new pcl::PointCloud<PointT>()
    );
    //typename pcl::PointCloud<PointT>::Ptr ground = new pcl::PointCloud<PointT>();
    typename pcl::PointCloud<PointT>::Ptr ground(
        new pcl::PointCloud<PointT>()
    );
    // Creating the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Extracting the given `inliers` from the `cloud`
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    // Storing extracted indices in output cloud
    extract.filter(*ground);
    std::cerr << "PointCloud representing the planar component: "
              << ground->width * ground->height 
              << " data points.\n";
    // Fetching the remaining non-extracted indices
    // outliers = extract.getRemovedIndices();
    // Copying over the remaining points into `obstacles` cloud
    extract.setNegative(true);
    extract.filter(*obstacles);
    std::cerr << "PointCloud representing the obstacles component: " 
              << obstacles->width * obstacles->height
              << " data points.\n"; 
    std::pair<
        typename pcl::PointCloud<PointT>::Ptr, 
        typename pcl::PointCloud<PointT>::Ptr
    > segResult(ground, obstacles);
    return segResult;
}


/** Segments the input cloud into two using the Point Cloud Library (PCL).
 * 
 * The input `cloud` is filtered and segmented into two such that a
 * ground plane (the road surface) is estimated by iteratively fitting
 * a planar surface using the RANSAC algorithm.
 * 
 * The parameters for this algorithm are provided in the input arguments,
 * `maxIterations` and `distanceThreshold`. 
 * 
 * For more information: 
 * https://pointclouds.org/documentation/tutorials/planar_segmentation.html.
 * 
 * @brief   Performs ground plane segmentation using Point Cloud Library (PCL).
 * @param   cloud               Point cloud to extract the ground plane from.
 * @param   maxIterations       Number of iterations to run the optimisation.
 * @param   distanceThreshold   Max distance from model to potential inlier.
 *                              As a rule of thumb, this should be slightly larger
 *                              than the resolution.
 * @returns Pair of point clouds returned from the `SeparateClouds` function,
 *          i.e., the segmented `ground` and the `obstacles` point clouds,
 *          contained in the `first` and `second` pair indices, respectively.
*/
template<typename PointT> std::pair<
    typename pcl::PointCloud<PointT>::Ptr, 
    typename pcl::PointCloud<PointT>::Ptr
> ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, 
    int maxIterations, 
    float distanceThreshold
) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    /** E1.2.2: Perform plane segmentation with PCL. **/
    // TODO:: Fill in this function to find inliers for the cloud.
    // Creating a new PCL segmentation class instance
    pcl::SACSegmentation<PointT> seg;
    // Creating intermediary objects consumed by the PCL algorithm
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};
    // Configuring the segmentation parameters and estimator
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setOptimizeCoefficients(true);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    // Segmenting the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset.\n";
    } 
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        endTime - startTime
    );
    std::cout << "Plane segmentation took " 
              << elapsedTime.count() 
              << " milliseconds.\n";
    std::pair<
        typename pcl::PointCloud<PointT>::Ptr, 
        typename pcl::PointCloud<PointT>::Ptr
    > segResult = SeparateClouds(inliers, cloud);
    return segResult;
}


/** Performs Euclidean clustering with the Point Cloud Library (PCL).
 *
 * Point clusters are extracted with `pcl::EuclideanClusterExtraction` class;
 * The clusters are populated with neighbouring points through a distance-based
 * seach query. Using a nearest neighbours algorithm, the candidate points are
 * searched with a KD-Tree structure, which restricts the possible points to
 * those in a general vicinty. This algorithm proposed in Rusu et al. (2010)
 * is described in detail for the Euclidean clustering problem:
 * https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html.
 *
 * @brief Performs Euclidean clustering with the Point Cloud Library (PCL).
 * @param   cloud            Point cloud to cluster.
 * @param   clusterTolerance Distance threshold (metres) to group points.
 * @param   minSize          Minimum points to be found in each cluster.
 * @param   maxSize          Maximum points to be found in each cluster.
 * @returns Point cloud containing the segmented point clusters.
 */
template<typename PointT> std::vector<
    typename pcl::PointCloud<PointT>::Ptr
> ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    float clusterTolerance,
    int minSize,
    int maxSize
) {
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<
        std::chrono::milliseconds
    >(endTime - startTime);
    std::cout << "Clustering took " 
              << elapsedTime.count() << " milliseconds,"
              << " and found " << clusters.size() << " clusters.\n";
    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
