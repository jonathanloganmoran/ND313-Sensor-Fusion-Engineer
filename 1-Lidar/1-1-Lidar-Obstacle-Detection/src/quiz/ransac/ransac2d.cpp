/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
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
			double d_ji = std::abs(
				A * p_j.x + B * p_j.y + C
			) / std::sqrt(
				std::pow(A, 2) + std::pow(B, 2)
			);
			std::cerr << ", d_ji = " << d_ji << ".\n";
			// Checking distance against threshold
			if (d_ji <= distanceTol) {
				// If distance is smaller than threshold,
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
							<< "numInliers: "
							<< bestNumInliersFound
							<< ").\n";
	}
	// Return indicies of inliers from "best" fitted line,
	// i.e., the line that "fit" the with most number of inliers
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();
	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	/*** E1.2.6: Modify the RANSAC model parameters. ***/
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
  int maxIterations = 10;
  double distanceTol = 0.5;
	std::unordered_set<int> inliers = Ransac(
    cloud, 
    maxIterations, 
    distanceTol
  );
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
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
