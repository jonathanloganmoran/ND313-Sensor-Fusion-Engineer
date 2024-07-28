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
#include <set>          // `SegmentPlaneCustom()` function
#include <stdlib.h>     // `srand`, `rand`
#include <time.h>       // `time`


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

/** Filters the point `cloud` to reduce the total number of data points.
 * 
 * Two 'downsampling' techniques are used in this function: voxel grid
 * point reduction, and region-based filtering. Each technique reduces
 * the total number of data points by either 'combining' the neighbouring
 * points within each "cell" into a single point, or by "grouping" the
 * space into sub-regions which can be further processed to reduce their
 * point counts.
 * 
 * NOTE: the `minPoint` and `maxPoint` arguments specify a region defined
 * by `Eigen::Vector4f` tuples (e.g., `Eigen::Vector4f(0.0, 0.0, 0.0, 1.0)`)
 * in which the given point `cloud` will be considered for filtering.
 * The points outside the region specified by these coordinate values will
 * be discarded during the filtering process.  
 * 
 * @brief Reduces the total number of data points in the input `cloud`.
 * @param filterRes The "cell" size to use for the voxel-based method.
 * @param minPoint Minimum coordinate of region to reduce.
 * @param maxPoint Maximum coordinate of region to reduce.
 * @returns The downsampled point cloud.
*/
template<typename PointT> typename pcl::PointCloud<
    PointT
>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, 
    float filterRes, 
    Eigen::Vector4f minPoint, 
    Eigen::Vector4f maxPoint
) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    /** E1.4.1: Filtering the point cloud with `pcl::VoxelGrid`. **/
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(
        new pcl::PointCloud<PointT>
    );
    // Creating the voxel-based filtering object
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    // Specifying the leaf size / "cell" dimensions
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);
    /** E1.4.2(a): Filtering the "scene" with `pcl::CropBox`. **/
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);
    // Defining the first region: the area of points to preserve
    pcl::CropBox<PointT> regionPreserved(true);
    regionPreserved.setMin(minPoint);
    regionPreserved.setMax(maxPoint);
    regionPreserved.setInputCloud(cloudFiltered);
    // Cropping the point cloud to the desired region (the "scene")
    regionPreserved.filter(*cloudRegion);
    /** E1.4.2(b): Filtering the "scene" with `pcl::CropBox`. **/
    // Creating a vector to store the indices determined to belong to the "roof"
    std::vector<int> indicesRoof;
    pcl::CropBox<PointT> roof(true);
    // Defining the points which form the area of the roof to filter out
    // NOTE: choosing non-zero valued vectors for `minPoint`, `maxPoint`;
    // These define the area of the region we wish to eliminate. 
    // CANDO: Modify these values to select a different area to eliminate points within.
    roof.setMin(
        Eigen::Vector4f(-1.5, -1.7, -1.0, 1)
    );
    roof.setMax(
        Eigen::Vector4f(2.6, 1.7, -0.4, 1)
    );
    roof.setInputCloud(cloudRegion);
    roof.filter(indicesRoof);
    // Populating the data structure with indices of the roof 
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for (int i = 0; i < indicesRoof.size(); i++) {
        inliers->indices.push_back(indicesRoof[i]);
    };
    // Extracting the roof indices (i.e., deleting them from point cloud)
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<
        std::chrono::milliseconds
    >(endTime - startTime);
    std::cout << "filtering took "
              << elapsedTime.count() << " milliseconds\n";
    return cloudRegion;
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

/** Segments the input cloud into two instances using standard library only.
 * 
 * The `SegmentPlaneCustom()` function "segments" the input cloud into two
 * instances: a `ground` plane cloud and an `obstacles`  cloud. The `ground`
 * plane, i.e., the "road surface", is estimated by iteratively fitting a
 * planar surface using the Random Sample Consensus (RANSAC) algorithm.
 * 
 * The parameters for this algorithm are provided in the input arguments,
 * `maxIterations` and `distanceThreshold`.
 * 
 * @param cloud Point cloud to extract the two instances from.
 * @param maxIterations Number of iterations to run the optimisation loop for.
 * @param distanceThreshold Max distance of current "model" to potential inlier.
 * @returns Pair of point cloud instances returned from `SeparateClouds()`,
 *      i.e., the segmented `ground` plane and the `obstacles` point clouds,
 *      accessible with the `.first` and `.second` dot-accessor, respectively. 
 */
template<typename PointT> std::pair<
    typename pcl::PointCloud<PointT>::Ptr,
    typename pcl::PointCloud<PointT>::Ptr
> ProcessPointClouds<PointT>::SegmentPlaneCustom(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    int maxIterations,
    float distanceThreshold
) {
    /** E1.5.1: Segmenting the point cloud into two instances. **/
    // Storing inliers of the "best fit" model (i.e., `ground` point indices)
    std::unordered_set<int> inliersResult;
    // Initialising the random number generator
    std::srand(time(NULL));
    /** Performing RASNAC model fitting for max iterations **/
    int bestNumInliersFound = std::numeric_limits<int>::min();
    for (int i = 0; i < maxIterations; i++) {
        std::cout << "Plane fitting, iteration: " << i << "\n";
        // Storing inliers of the current plane ("model")
        std::unordered_set<int> inliersTemp;
        // Sampling three points at random
        int numPoints = (int)cloud->size();
        // Using `set` to prevent "duplicate" anchor points
        std::set<int> anchorPoints;
        while (anchorPoints.size() < 3) {
            anchorPoints.insert(
                rand() % numPoints
            );
        }
        /* Catching any errors with selecting unique anchor points */
        if (anchorPoints.empty()) {
            std::cerr << "Error; cannot form co-linear vectors, "
                      << "Must have three unique points.\n";
            return inliersResult;
        }
        else if (anchorPoints.size()) {
            std::cerr << "Error; not enough unique points in dataset.\n";
            return inliersResult;
        }
        // Fetching the indices of the three unique anchor points found
        std::set<int>::iterator idx = anchorPoints.begin();
        int pointIdx1 = *idx; idx++;
        int pointIdx2 = *idx; idx++;
        int pointIdx3 = *idx;
        // Fetching the anchor points (i.e., their 3D point values)
        pcl::PointXYZI p1 cloud->points[pointIdx1];
        pcl::PointXYZI p2 cloud->points[pointIdx2];
        pcl::PointXYZI p3 cloud->points[pointIdx3];
        /* "Fitting" the equation of the plane to the three points. */
        // First, forming two vectors originating from `p1`
        double v1[3] = {
            p2.x - p1.x,
            p2.y - p1.y,
            p2.z - p1.z
        };
        double v2[3] = {
            p3.x - p1.x,
            p3.y - p1.y,
            p3.z - p1.z
        };
        // Second, taking cross-product to form "normal vector" `v1xv2`
        double v1xv2[3] = {
            (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y),
            (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z),
            (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)
        };
        // Finally, extracting values of the coefficients of the plane
        double A = v1xv2[0];
        double B = v1xv2[1];
        double C = v1xv2[2];
        // And determining the final coefficient `D` with arbitrary point
        double D = -(
            A * p1.x + B * p1.y + C * p1.z
        );
        /* Computing point-plane distance over all points */
        // First, creating counter to get number of current inliers
        // "Inliers" here refers to point(s) with a to-plane distance
        // less than the given threshold value.
        int numInliersCurrent = 0;
        std::cout << "Point-plane distance computation\n";
        for (int j = 0; j < numPoints; j++) {
            // CANDO: Comment out console logging for less "clutter"
            std::cout << "Iteration " << j << ": "
                      << "`numInliersCurrent` = " << numInliersCurrent
                      << ", `p1` = " << p1
                      << ", `p2` = " << p2
                      << ", `p3` = " << p3;
            // Fetching point candidate "at random"
            int pointIdxj = rand() % numPoints;
            // Checking if point candidate is already an anchor point
            // i.e., one of the point(s) used to fit the plane
            if ((pointIdxj == pointIdx1)
                || (pointIdxj == pointIdx2)
                || (pointIdxj == pointIdx3) 
            ) {
                // Fetching point value to print in console log
                pcl::PointXYZI p_err cloud->points[pointIdxj];
                // Throw error; randomly-selected point is an anchor point
                // CANDO: Comment out console logging for less "clutter"
                std::cerr << "Model iteration: " << i
                          << ", point iteration: " << j
                          << ", anchor point encountered at index: "
                          << pointIdxj << " with values: {x, y, z, I} = "
                          << "{" << p_err.x
                          << ", " << p_err.y 
                          << ", " << p_err.z 
                          << ", " << p_err.z
                          << ", " << p_err.I << "}.\n";
                // CANDO: Skip adding anchor to set
                // To avoid divide-by-zero errrors
                continue;
                // Additionally, handle "other" possible edge cases, such as:
                // CANDO: Case (1) "Coincident point" — Point is on plane.
                // CANDO: Case (2) "Parallel plane" — Plane is parallel to coordinate axes.
                // CANDO: Case (3) "Numerical instability" — Very small denominator values.
                // CANDO: Case (4) "Infinity distance" — plane parallel to point vector. 
            }
            pcl::PointXYZI p_j = cloud->points[pointIdxj];
            // CANDO: Comment out console logging for less "clutter"
            std::cout << ", `p_j` = " << p_j;
            // Calculating the distance from point to plane
            double d_j_dot_v1xv2 = std::fabs(
                A * p_j.x + B * p_j.y + C * p_j.z + D
            ) / std::sqrt(
                std::pow(A, 2) + std::pow(B, 2) + std::pow(C, 2)
            );
            // Checking computed distance against threshold
            if (d_j_dot_v1xv2 <= distanceTol) {
                // Distance is within tolerated limit
                // i.e., Point is considered an "inlier"
                numInliersCurrent += 1;
                inliersTemp.insert(pointIdxj);
            }
        } // Repeat for all remaining points in point cloud
        /* Checking if current model was "best" found */
        if (numInliersCurrent >= bestNumInliersFound) {
            // Update the "best" inlier set to be this current one
            inliersResult = inliersTemp;
            bestNumInliersFound = numInliersCurrent;
        }
        // Otherwise, clear this model's inlier set and repeat with new plane
        inliersTemp.clear();
        // Reset number of inliers found for the next model iteration
        numInliersCurrent = 0;
    } // Repeat model fitting for maximum number of iterations
    /* End of model fitting */
    // Checking if we obtained any inliers from the "best" run (sanity check)
    if (bestNumInliersFound <= 0 || inliersResult.empty()) {
        // No inliers found; or, error has occurred.
        std::cerr << "Error has occurred; no inliers found ("
                  << "`bestNumInliersFound` = " << bestNumInliersFound
                  << ").\n";
    } // Otherwise, a valid "inlier" set should have been obtained.
    // Obtained the indices of the inliers found from the "best" fit model,
    // i.e., the ground plane that "fit" the most number of inliers.
    /* "Separating" the input `cloud` into two instances */
    // Copying the found "inliers" (the set of integer-value indices) into a PCL object
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    inliers->indices = inliersResult;
    // With the set of inliers, "split" the input `cloud` into two instances
    std::pair<
        typename pcl::PointCloud<PointT>::Ptr, 
        typename pcl::PointCloud<PointT>::Ptr
    > segResult = SeparateClouds(
        inliers, 
        cloud
    );
    // Returning the two point cloud instances in a pair,
    // the first (`inliersResult.first`) is the `ground`,
    // the second (`inliersResult.second`) are the `obstacles`.
    return segResult;
}

/** Segments the input cloud into two using the Point Cloud Library (PCL).
 * 
 * The `SegmentPlane()` function "segments" the input cloud into two instances:
 * a `ground` plane cloud and an `obstacles` cloud. The `ground` plane, i.e.,
 * the "road surface", is estimated by iteratively fitting a planar surface
 * with the RANSAC algorithm provided by the Point Cloud Library (PCL).
 * 
 * The parameters for this algorithm are provided in the input arguments,
 * `maxIterations` and `distanceThreshold`. 
 * 
 * For more information: 
 * https://pointclouds.org/documentation/tutorials/planar_segmentation.html.
 * 
 * @brief   Performs ground plane segmentation using Point Cloud Library (PCL).
 * @param cloud Point cloud to extract the two instances from.
 * @param maxIterations Number of iterations to run the optimisation loop for.
 * @param distanceThreshold Max distance of current "model" to potential inlier,
*       As a rule of thumb, this should be slightly larger than the resolution.
 * @returns Pair of point cloud instances returned from `SeparateClouds()`,
 *      i.e., the segmented `ground` plane and the `obstacles` point clouds,
 *      accessible with the `.first` and `.second` dot-accessor, respectively. 
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
 * @param   cloud            Point cloud to cluster. Assumed to be filtered.
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
    /*** E1.3.1: Euclidean clustering with PCL. ***/
    // Creating the KD-Tree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(
        new pcl::search::KdTree<PointT>
    );
    // Creating the Euclidean clustering class instance
    pcl::EuclideanClusterExtraction<PointT> ec;
    // Setting the input cloud for the KD-Tree
    // NOTE: We assume the ground plane has been "filtered" out
    tree->setInputCloud(cloud);
    // Configuring the clustering parameters
    std::vector<pcl::PointIndices> clusterIndices;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);
    // Performing the clustering with Euclidean distance
    for (const auto& cluster : clusterIndices) {
        // Creating a new point cloud instance for the current cluster
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(
            new pcl::PointCloud<PointT>
        );
        for (const auto& idx : cluster.indices) {
            // Copying over the indices of the current cluster
            cloudCluster->push_back(
                (*cloud)[idx]
            );
        }
        // Setting the cluster parameters
        cloudCluster->width = cloudCluster->size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        std::cout << "PointCloud representing the Cluster: "
                  << cloudCluster->size() << " data points.\n";
        // Adding cluster to return vector
        clusters.push_back(cloudCluster);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<
        std::chrono::milliseconds
    >(endTime - startTime);
    std::cout << "Clustering took " 
              << elapsedTime.count() << " milliseconds,"
              << " and found " << clusters.size() << " clusters.\n";
    return clusters;
}

/** Computes a 3D bounding box for the given point cloud.
 * 
 * Returns the intersection points of a rectangular prism formed by
 * the 3D point cloud's minimum and maximum points found along each
 * axis. 
 * 
 * @brief Uses the `pcl::getMinMax3D()` function to obtain bounding box.
 * @param cluster The 3D point cloud instance to form bounding box for.
 * @returns The 3D `Box` with coordinates of each intersection point.
 */
template<typename PointT> Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster
) {
    /** E1.3.6: Computing bounding box for the point cluster **/
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
