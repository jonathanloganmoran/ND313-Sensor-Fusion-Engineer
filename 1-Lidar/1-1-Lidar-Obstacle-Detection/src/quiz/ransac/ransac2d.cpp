/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <set>			// For ordered `set` (in `bits/stdc++.h`).

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

/** Performs planar fitting with RANSAC algorithm.
 * 
 * The input point cloud is expected to contain 3D points, such that the
 * $z$-axis values are non-zero. This function attempts to fit a 3D "model" to
 * a set of points contained in the point cloud. In this case, the model is
 * a 3D planar surface. Any points within a certain distance to the plane will
 * be considered as "inlier" points. The distance threshold (`distanceTol`)
 * and the number of model fitting iterations (`maxIterations`) are
 * configurable as input arguments to this function.
 * 
 * @brief	Performs planar fitting with RANSAC over a 3D point cloud.
 * @param		cloud						Input cloud to cluster, assumes all points are 3D.
 * @param		maxIterations 	Number of times to "fit" a line to the points.
 * @param		distanceTol			Distance to a given point and the current plane for
 * 													the point to be considered an "inlier".
 * @returns	The set of indices of the "inlier" points which belonged to the
 * 				  line of "best fit" (i.e., line with most number of inliers).
 */
std::unordered_set<int> RansacPlane(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
		int maxIterations, 
		float distanceTol
) {
	// Storing the inliers of the "best fit" model
	std::unordered_set<int> inliersResult;
	// Initialising the random number generator
	std::srand(time(NULL));
	/*** E1.2.7: Perform RANSAC for 3D plane fitting. ***/
	int bestNumInliersFound = std::numeric_limits<int>::min();
	/** Performing model fitting for max iterations **/
	for (int i = 0; i < maxIterations; i++) {
		// Storing the inliers of the current plane
		std::unordered_set<int> inliersTemp;
		/** Sampling three points "at random" **/
		// Randomly sample subset of points to fit plane
		int numPoints = (int)cloud->size();
		// Selecting three point indices at "random"
		// TODO: Check for co-linearity with matrix determinant calculation
		// CANDO: Use `pcl::determinant3x3Matrix()` with `Matrix` type;
		// Will need: `#include <pcl/common/eigen.h>`.
		std::set<int> anchorPoints;
		while(anchorPoints.size() < 3) {
			// Making sure the three "anchor" points are not equal,
			// Since the `std::set` will not allow for duplicates.
			anchorPoints.insert(
				rand() % numPoints
			);
		}
		if (anchorPoints.empty()) {
			std::cerr << "Error; cannot form co-linear vectors, "
							  <<  "Must have three unique points.";
			return inliersResult;
		}
		else if (anchorPoints.size() < 3) {
			std::cerr << "Error; not enough unique points found.";
			return inliersResult;
		}
		// Fetching the indices of the three anchor points to use later
		std::set<int>::iterator idx = anchorPoints.begin();
		int pointIdx1 = *idx; idx++;
		int pointIdx2 = *idx; idx++;
		int pointIdx3 = *idx;
		// Fetching the three randomly-selected anchor points for the plane
		pcl::PointXYZ p1 = cloud->points[pointIdx1];
		pcl::PointXYZ p2 = cloud->points[pointIdx2];
		pcl::PointXYZ p3 = cloud->points[pointIdx3];
		/** "Fitting" the equation of a plane to the three points **/
		// First, we form the two vectors originating from `p1`:
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
		// Secondly, with the cross product we form a normal vector to the plane,
		// Such that $v_{1} \times v_{2} = <i, j, k>$:
		double v1xv2[3] = {
			(p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y),
			(p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z),
			(p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)
		};
		// Lastly, we extract the values of the coefficients of the plane:
		double A = v1xv2[0];
		double B = v1xv2[1];
		double C = v1xv2[2];
		double D = -(
			v1xv2[0] * p1.x + v1xv2[1] * p1.y + v1xv2[2] * p1.z
		);
		/** Computing point-plane distance over all points **/
		// Counting number of current inliers found
		// "Inlier" here means a point with a distance to the plane
		// less than the given distance threshold value
		int numInliersCurrent = 0;
		for (int j = 0; j < numPoints; j++) {
			std::cerr << "Iteration " << j << ": "
								<< "numInliersCurrent = " << numInliersCurrent
								<< ", p1 = " << p1
								<< ", p2 = " << p2
								<< ", p3 = " << p3;
			// Fetching point "at random"
			int pointIdxj = rand() % numPoints;
			// Checking if one of the anchor points,
			// i.e., the point(s) used to fit the plane
			if ((pointIdxj == pointIdx1)
			  || (pointIdxj == pointIdx2)
				|| (pointIdxj == pointIdx3)  
			) {
				// Randomly-selected point is an anchor point,
				std::cerr << "\nAnchor point encountered.\n";
				// Add anchor point to set of inliers
				inliersTemp.insert(j);
				// Skip distance calculation to avoid divide-by-zero errors
				// CANDO: Case (1) "Coincident" point - point is on plane.
				// CANDO: Case (2) Parallel plane - plane is parallel to coordinate axes.
				// CANDO: Case (3) Numerical instability - very small denominator values.
				// CANDO: Case (4) Infinity distance - plane parallel to point vector.
				continue;
			}
			pcl::PointXYZ p_j = cloud->points[pointIdxj];
			std::cerr << ", p_j = " << p_j;
			// Calculating distance from point $j$ to plane $v_{1} \times v_{2}$
			double d_jv1xv2 = std::fabs(
				A * p_j.x + B * p_j.y + C * p_j.z + D
			) / std::sqrt(
				std::pow(A, 2) + std::pow(B, 2) + std::pow(C, 2)
			);
			// Checking distance against threshold
			if (d_jv1xv2 <= distanceTol) {
				// Distance is within threshold,
				// Point is considered to be an "inlier"
				numInliersCurrent += 1;
				inliersTemp.insert(pointIdxj);
			}
		} // Repeat for all remaining points in point cloud
		// Checking if this plane fit the "best" (most) number of points
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
	// Checking if we obtained a "valid" result
	if (bestNumInliersFound <= 0) {
		// No inliers found; or, error has occurred
		std::cerr << "Error has occurred; no inliers found ("
							<< "bestNumInliersFound: " << bestNumInliersFound
							<< ").\n";
	}
	// Return indices of inliers from "best" fitted plane,
	// i.e., the plane that "fit" the most number of inliers.
	return inliersResult;
}


/** Performs 2D line fitting with RANSAC algorithm.
 * 
 * The input point cloud is expected to contain 2D points, such that the
 * $z$-axis values are zero. This function attempts to fit a 2D "model" to
 * a set of points contained in the point cloud. In this case, the model is
 * a 2D line. Any points within a certain distance to the line will be
 * considered as "inlier" points. The distance threshold (`distanceTol`)
 * and the number of model fitting iterations (`maxIterations`) are
 * configurable as input arguments to this function.
 * 
 * @brief	Performs 2D line fitting with RANSAC over a 2D point cloud.
 * @param		cloud						Input cloud to cluster, assumes all points are 2D.
 * @param		maxIterations 	Number of times to "fit" a line to the points.
 * @param		distanceTol			Distance to a given point and the current line for
 * 													the point to be considered an "inlier".
 * @returns	The set of indices of the "inlier" points which belonged to the
 * 				  line of "best fit" (i.e., line with most number of inliers).
 */
std::unordered_set<int> Ransac(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
		int maxIterations, 
		float distanceTol
) {
	// Storing the inliers of the "best fit" model
	std::unordered_set<int> inliersResult;
  // Initialising the random number generator
	srand(time(NULL));
  /*** E1.2.5: Perform RANSAC for 2D line fitting. ***/
	// TODO: Fill in this function
	// Storing greatest number of inliers found
	int bestNumInliersFound = std::numeric_limits<int>::min();
	/** Performing model fitting for max iterations **/
	for (int i = 0; i < maxIterations; i++) {
		// Randomly sample subset and fit line
		// Storing the inliers of this current line
		std::unordered_set<int> inliersTemp;
		/** Sampling two points "at random" **/
		// Getting number of points total in point cloud
		int numPoints = (int)cloud->size();
		// Selecting two point indices "at random" 
		int pointIdx1 = rand() % numPoints;
		int pointIdx2 = rand() % numPoints;
		// Fetching the two randomly-selected points
		pcl::PointXYZ p1 = cloud->points[pointIdx1];
		pcl::PointXYZ p2 = cloud->points[pointIdx2];
		/** "Fitting" the equation of a line to the two points **/
		double A = p1.y - p2.y;
		double B = p2.x - p1.x;
		double C = (p1.x * p2.y) - (p1.y * p2.x);
		/** Computing point-line distance over all points **/
		// Counting number of current inlier points with distances less than threshold
		int numInliersCurrent = 0;
		for (int j = 0; j < numPoints; j++) {
			std::cerr << "Iteration " << j << ": "
								<< "numInliersCurrent = " << numInliersCurrent
								<< ", p1 = " << p1
								<< ", p2 = " << p2;
			// Fetching point "at random"
			int pointIdxj = rand() % numPoints;
			// Checking if one of anchor points,
			// i.e., point used to fit line.
			if ((pointIdxj == pointIdx1)
				  || (pointIdxj == pointIdx2)
			) {
				// Randomly-selected point is an anchor point,
				std::cerr << "\nAnchor point encountered.\n";
				// Add anchor point to set of inliers
				inliersTemp.insert(j);
				// Skip distance calculation to avoid divide-by-zero errors
				continue;
			}
			pcl::PointXYZ p_j = cloud->points[pointIdxj];
			std::cerr << ", p_j = " << p_j;
			// Calculating distance from point $j$ to line $i$
			double d_ji = std::fabs(
				A * p_j.x + B * p_j.y + C
			) / std::sqrt(
				std::pow(A, 2) + std::pow(B, 2)
			);
			std::cerr << ", d_ji = " << d_ji << ".\n";
			// Checking distance against threshold
			if (d_ji <= distanceTol) {
				// Distance is within threshold,
				// Point is considered to be an "inlier"
				numInliersCurrent += 1;
				// Store point index in the current inliers set
				inliersTemp.insert(pointIdxj);
			}
		}
		// Checking if this line fit the "best" (most) number of points
		if (numInliersCurrent >= bestNumInliersFound) {
			// Update the "best" inlier set to be this current one
			inliersResult = inliersTemp;
			bestNumInliersFound = numInliersCurrent;
		}
		// Otherwise, clear this model's inlier set and repeat with new line
		inliersTemp.clear();
		// Reset number of inliers found for the next model iteration
		numInliersCurrent = 0;
	}  // Repeat next model iteration
	// Checking if we obtained a "valid" result
	if (bestNumInliersFound <= 0) {
		// No inliers found; or, error has occurred.
		std::cerr << "Error has occurred; no inliers found ("
							<< "bestNumInliersFound: " << bestNumInliersFound
							<< ").\n";
	}
	// Return indices of inliers from "best" fitted line,
	// i.e., the line that "fit" the most number of inliers.
	return inliersResult;
}


int main() {
	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();
	/*** E1.2.6 / E1.2.8: Modify the RANSAC model parameters. ***/
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
  int maxIterations = 10;
  double distanceTol = 0.5;
	/*** E1.2.5: Perform RANSAC for 2D line fitting. ***/
	// First, we create the 2D data needed for this experiment using the helper function
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	// Then, we run the RANSAC for 2D line fitting function:
	std::unordered_set<int> inliers = Ransac(
    cloud, 
    maxIterations, 
    distanceTol
  );
	/*** E1.2.7: Perform RANSAC for 3D plane fitting. ***/
  // First, we create the 3D data needed for this experiment using the helper function
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
  // Then, we run the RANSAC for 3D plane fitting function:
	std::unordered_set<int> inliers = RansacPlane(
    cloud, 
    maxIterations, 
    distanceTol
  );
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
}
