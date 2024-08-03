/* ----------------------------------------------------------------------------
  * Project "1.1: LiDAR Obstacle Detection"
  * Authors     : Aaron Brown et al.
  *
  * Modified by : Jonathan Logan Moran (jonathan.moran107@gmail.com).
  *
  * Purpose of this file: Implements quiz questions related to the K-D Tree.
  * 		This corresponds to Exercises 1.3.3-5 and Exercises 1.5.2 of 
  * 		Course 1: Lidar in the Sensor Fusion Nanodegree offered by Udacity.
  * ---------------------------------------------------------------------------
  */

#include "kdtree.h"
#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>


/** Refreshes the PCL Viewer canvas for the 2D K-D Tree visualisation task.
 * 
 * NOTE: The `zoom` value is passed to the 
 * 	`pcl::visualization::PCLVisualizer::setCameraPosition()`
 * function and is used to "zoom" along the z-axis of the viewer.
 * 
 * @param window Region in which to draw the "box" around.
 * @param zoom The $z$-coordinate value of the PCL "camera".
 * @returns The PCL Viewer instance with a "fresh" canvas configuration.
 */
pcl::visualization::PCLVisualizer::Ptr initScene(
	Box window, 
	int zoom
) {
	pcl::visualization::PCLVisualizer::Ptr viewer(
		new pcl::visualization::PCLVisualizer("2D Viewer")
	);
	viewer->setBackgroundColor(0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  	viewer->addCoordinateSystem(1.0);
  	viewer->addCube(
		window.x_min, 
		window.x_max, 
		window.y_min, 
		window.y_max, 
		0, 
		0, 
		1, 
		1, 
		1, 
		"window"
	);
  	return viewer;
}

/** Constructs a PCL PointCloud from the vector 3D coordinate `points`.
 *
 * This function expects `points` to be a vector of floating point-valued
 * vectors, each a set of 3D coordinate values.
 * 
 * Each 3D coordinate pair is casted as a `pcl::PointXYZ` instance.
 * 
 * @param points Vector of 3D point coordinate value(s).
 * @returns PCL Point Cloud instance created from the point coordinates.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D(
	std::vector<std::vector<float>> points
) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
		new pcl::PointCloud<pcl::PointXYZ>()
	);
  	for (int i = 0; i < points.size(); i++) {
  		pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = points[i][2];
  		cloud->points.push_back(point);
  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;
  	return cloud;
}

/** Constructs a PCL PointCloud from the vector 2D coordinate `points`.
 *
 * This function expects `points` to be a vector of floating point-valued
 * vectors, each a set of 2D coordinate values.
 * 
 * Each 2D coordinate pair is casted as a `pcl::PointXYZ` instance such that
 * the $z$-axis value is `0`. 
 * 
 * @param points Vector of 2D point coordinate value(s).
 * @returns PCL Point Cloud instance created from the point coordinates.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(
	std::vector<std::vector<float>> points
) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
		new pcl::PointCloud<pcl::PointXYZ>()
	);
  	for (int i = 0; i < points.size(); i++) {
  		pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = 0;
  		cloud->points.push_back(point);
  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;
  	return cloud;
}

/** Visualises the 3D K-D Tree using Point Cloud Library (PCL).
 * 
 * Iterates recurisvely over the 3D K-D Tree starting with the root `node`.
 * This function visualises the coordinate values as "points" on the plane.
 * The axes of the tree (i.e., $x$-, $y$- or $z$-axis) alternate with each
 * iteration and their "splitting" values are visualised as 'lines' onto the
 * PCL canvas. Each axis being "split" on is assigned a colour and rendered
 * accordingly.
 * 
 * @param node The root node of the 3D K-D Tree to traverse.
 * @param viewer The PCL canvas to render the elements onto.
 * @param window The `Box` struct visualising the sub-region being examined.
 * @param iteration Counter indexing the number of "splits" made so far.
 * @param depth Which "level" (axis) currently examined in the 3D K-D Tree.
 */
void renderTree3D(
	Node *node, 
	pcl::visualization::PCLVisualizer::Ptr &viewer, 
	Box window, 
	int &iteration, 
	uint depth = 0
) {
	if (node != NULL) {
		Box upperWindow = window;
		Box lowerWindow = window;
		// split on x axis
		if (depth % 3 == 0) {
			viewer->addLine(
				pcl::PointXYZ(node->point[0], window.y_min, 0),
				pcl::PointXYZ(node->point[0], window.y_max, 0),
				0,
				0,
				1,
				"line" + std::to_string(iteration)
			);
			lowerWindow.x_max = node->point[0];
			upperWindow.x_min = node->point[0];
		}
		// split on y axis
		else if (depth % 3 == 1) {
			viewer->addLine(
				pcl::PointXYZ(window.x_min, node->point[1], 0),
				pcl::PointXYZ(window.x_max, node->point[1], 0),
				1,
				0,
				0,
				"line" + std::to_string(iteration)
			);
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		}
		// split on z axis
		else if (depth % 3 == 2) {
			viewer->addLine(
				pcl::PointXYZ(window.x_min, 0, node->point[2]),
				pcl::PointXYZ(window.x_max, 0, node->point[2]),
				1,
				0,
				0,
				"line" + std::to_string(iteration)
			);
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		}
		iteration++;
		renderTree3D(
			node->left, 
			viewer, 
			lowerWindow, 
			iteration, 
			depth + 1
		);
		renderTree3D(
			node->right, 
			viewer, 
			upperWindow, 
			iteration, 
			depth + 1
		);
	}
}

/** Visualises the 2D K-D Tree using Point Cloud Library (PCL).
 * 
 * Iterates recurisvely over the 2D K-D Tree starting with the root `node`.
 * This function visualises the coordinate values as "points" on the plane.
 * The axes of the tree (i.e., $x$- or $y$-axis) alternate with each iteration
 * and their "splitting" values are visualised as 'lines' onto the PCL canvas.
 * Each axis being "split" on is assigned a colour and rendered accordingly.
 * 
 * @param node The root node of the K-D Tree to traverse.
 * @param viewer The PCL canvas to render the elements onto.
 * @param window The `Box` struct visualising the sub-region being examined.
 * @param iteration Counter indexing the number of "splits" made so far.
 * @param depth Which "level" (axis) currently examined in the K-D Tree.
 */
void renderTree2D(
	Node *node, 
	pcl::visualization::PCLVisualizer::Ptr &viewer, 
	Box window, 
	int &iteration, 
	uint depth = 0
) {
	if (node != NULL) {
		Box upperWindow = window;
		Box lowerWindow = window;
		// split on x axis
		if (depth % 2 == 0) {
			viewer->addLine(
				pcl::PointXYZ(node->point[0], window.y_min, 0),
				pcl::PointXYZ(node->point[0], window.y_max, 0),
				0,
				0,
				1,
				"line" + std::to_string(iteration)
			);
			lowerWindow.x_max = node->point[0];
			upperWindow.x_min = node->point[0];
		}
		// split on y axis
		else {
			viewer->addLine(
				pcl::PointXYZ(window.x_min, node->point[1], 0),
				pcl::PointXYZ(window.x_max, node->point[1], 0),
				1,
				0,
				0,
				"line" + std::to_string(iteration)
			);
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		}
		iteration++;
		renderTree2D(
			node->left, 
			viewer, 
			lowerWindow, 
			iteration, 
			depth + 1
		);
		renderTree2D(
			node->right, 
			viewer, 
			upperWindow, 
			iteration, 
			depth + 1
		);
	}
}

/** Euclidean clustering helper function; populates `cluster` with 2D points.
 * 
 * @brief Performs Euclidean clustering for the given 2D point `idx`.
 * @param idx Index of the current point in `points` to "process".
 * @param points Vector of all points in point cloud to cluster.
 * @param c Current cluster to populate with neighbouring points.
 * @param visited Tracks whether a given point index has been examined.
 * @param tree K-D Tree to search for point neighbours in.
 * @param distanceTol Distance tolerance (in metres) used to bisect search space.
 */
std::vector<std::vector<int>> cluster3D(
	int idx,
	const std::vector<std::vector<float>> &points,
	std::vector<int> &c,
	std::vector<bool> &visited,
	KdTree *tree,
	float distanceTol
) {
	// Marking current point as "visited"
	visited[idx] = true;
	// Adding the point index to the "cluster" (assignment)
	c.push_back(idx);
	// Performing the K-D Tree search for neighbouring points
	std::vector<int> idxs = tree->search(
		points[idx], 
		distanceTol
	);
	// Recursively "building out" the K-D Tree for each neighbouring point
	for (int i = 0; i < idxs.size(); i++) {
		// Fetching the next neighbouring point's index
		int idxPoint = idxs[i];
		if (!visited[idxPoint]) {
			cluster(
				idxPoint,
				points,
				c,
				visited,
				tree,
				distanceTol
			);
		}
	}
}

/** Groups 2D `points` into individual cluster indices based on their proximity.
 * 
 * @brief Performs Euclidean clustering on the input `points`.
 * @param points Set of point coordinates to group into clusters.
 * @param tree K-D Tree instance to "fill out" with neighbouring points.
 * @param distanceTol Distance tolerance (in metres) used to bisect search space.
 */
std::vector<std::vector<int>> euclideanCluster(
	const std::vector<std::vector<float>> &points, 
	KdTree *tree, 
	float distanceTol
) {
	/** E1.3.5: Euclidean Clustering with the K-D Tree **/
	std::vector<std::vector<int>> clusters;
	// Creating list of "processed" indices
	std::vector<bool> visited{points.size(), false};
	// Forming "clusters" for each point in the point cloud
	for (int i = 0; i < points.size(); i++) {
		// Skipping point if already processed
		if (visited[i]) {
			continue;
		}
		// Creating a new `cluster` and finding neighbouring points
		std::vector<int> c;
		cluster(i, points, c, visited, tree, distanceTol);
		// Adding resulting cluster to vector
		clusters.push_back(c);
	}
	return clusters;
}

/** Euclidean clustering helper function; populates `cluster` with 3D points.
 * 
 * @brief Performs Euclidean clustering for the given 3D point `idx`.
 * @param idx Index of the current 3D points in `points` to "process".
 * @param points Vector of all 3D points in point cloud to cluster.
 * @param c Current cluster to populate with neighbouring points.
 * @param visited Tracks whether a given point index has been examined.
 * @param tree 3D K-D Tree to search for point neighbours in.
 * @param distanceTol Distance tolerance (in metres) used to bisect search space.
 */
std::vector<std::vector<int>> cluster3D(
	int idx,
	const std::vector<std::vector<float>> &points,
	std::vector<int> &c,
	std::vector<bool> &visited,
	KdTree3D *tree,
	float distanceTol
) {
	// Marking current point as "visited"
	visited[idx] = true;
	// Adding the point index to the "cluster" (assignment)
	c.push_back(idx);
	// Performing the K-D Tree search for neighbouring points
	std::vector<int> idxs = tree->search(
		points[idx], 
		distanceTol
	);
	// Recursively "building out" the K-D Tree for each neighbouring point
	for (int i = 0; i < idxs.size(); i++) {
		// Fetching the next neighbouring point's index
		int idxPoint = idxs[i];
		if (!visited[idxPoint]) {
			cluster3D(
				idxPoint,
				points,
				c,
				visited,
				tree,
				distanceTol
			);
		}
	}
}

/** Groups 3D `points` into individual cluster indices based on their proximity.
 * 
 * @brief Performs Euclidean clustering on the input 3D `points`.
 * @param points Set of 3D point coordinates to group into clusters.
 * @param tree 3D K-D Tree instance to "fill out" with neighbouring points.
 * @param distanceTol Distance tolerance (in metres) used to bisect search space.
 */
std::vector<std::vector<int>> euclideanCluster3D(
	const std::vector<std::vector<float>> &points, 
	KdTree3D *tree, 
	float distanceTol
) {
	/** E1.5.3: Euclidean Clustering with the 3D K-D Tree **/
	std::vector<std::vector<int>> clusters;
	// Creating list of "processed" indices
	std::vector<bool> visited{points.size(), false};
	// Forming "clusters" for each point in the point cloud
	for (int i = 0; i < points.size(); i++) {
		// Skipping point if already processed
		if (visited[i]) {
			continue;
		}
		// Creating a new `cluster` and finding neighbouring points
		std::vector<int> c;
		cluster3D(i, points, c, visited, tree, distanceTol);
		// Adding resulting cluster to vector
		clusters.push_back(c);
	}
	return clusters;
}

/** Orchestrates the 2D K-D Tree visualiser programme.
 * 
 * Here, a K-D Tree (`struct KdTree`) is instantiated, then populated with
 * 2D point values which are converted to a compatible PCL Point Cloud instance.
 * Then, the K-D Tree and its respective 2D coordinate values are rendered onto
 * the PCL Viewer, which contains the elements needed to visually represent
 * the `search()` function, i.e., axis "splitting" and sub-region "searching".
 * The programme also performs `euclideanClustering()` so that the "clusters"
 * are determined and assigned unique colours so that they (and their
 * respective point values) are visually distinct from each other.
 * Each "cluster" of points is rendered using the `renderPointCloud()` function.
 */
int main() {
	/** Running the 2D or 3D K-D Tree programme. **/
	// Set `render2D` to `false` for E1.5.2
	bool render2D = false;
	if (render2D) {
		/** E1.3.4-5: K-D Tree in 2D. **/
		// Create viewer
		Box window;
		window.x_min = -10;
		window.x_max = 10;
		window.y_min = -10;
		window.y_max = 10;
		window.z_min = 0;
		window.z_max = 0;
		pcl::visualization::PCLVisualizer::Ptr viewer = initScene(
			window, 
			25
		);
		/** Creating 2D point data **/
		// Test Case 1
		std::vector<std::vector<float>> points = {
			{-6.2, 7.0}, {-6.3, 8.4}, {-5.2, 7.1}, {-5.7, 6.3},
			{7.2, 6.1}, {8.0, 5.3}, {7.2, 7.1}, {0.2, -7.1},
			{1.7, -6.9}, {-1.2, -7.2}, {2.2, -8.9}
		};
		// Test Case 2
		// std::vector<std::vector<float>> points = {
		// 	{-6.2, 7}, {-6.3, 8.4}, {-5.2, 7.1}, {-5.7, 6.3}
		// };
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(
			points
		);
		KdTree* tree = new KdTree;
		for (int i = 0; i < points.size(); i++) {
			tree->insert(
				points[i],
				i
			); 
		}
		int it = 0;
		renderTree2D(
			tree->root,
			viewer,
			window, 
			it
		);
		std::cout << "Test Search\n";
		/** E1.3.4: Searching the K-D Tree for nearest neighbours **/
		std::vector<int> nearby = tree->search(
			{-6, 7}, 
			3.0
		);
		for (int index : nearby) {
			std::cout << index << ",";
		}
		std::cout << std::endl;
		// Time segmentation process
		auto startTime = std::chrono::steady_clock::now();
		/** E1.3.5: Euclidean Clustering with the K-D Tree **/
		std::vector<std::vector<int>> clusters = euclideanCluster(
			points, 
			tree, 
			3.0
		);
		auto endTime = std::chrono::steady_clock::now();
		auto elapsedTime = std::chrono::duration_cast<
			std::chrono::milliseconds
		>(endTime - startTime);
		std::cout << "clustering found " << clusters.size()
					<< " and took " << elapsedTime.count() << " milliseconds\n";
		// Render clusters
		int clusterId = 0;
		std::vector<Color> colors = {
			Color(1, 0, 0), 
			Color(0, 1, 0), 
			Color(0, 0, 1)
		};
		for (std::vector<int> cluster : clusters) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(
				new pcl::PointCloud<pcl::PointXYZ>()
			);
			for (int indice : cluster) {
				clusterCloud->points.push_back(
					pcl::PointXYZ(
						points[indice][0],
						points[indice][1],
						0
					)
				);
			}
			renderPointCloud(
				viewer, 
				clusterCloud,
				"cluster" + std::to_string(clusterId),
				colors[clusterId % 3]
			);
			++clusterId;
		}
		if (clusters.size() == 0) {
			renderPointCloud(
				viewer,
				cloud,
				"data"
			);
		}
		while (!viewer->wasStopped()) {
			viewer->spinOnce();
		}
	}
	else if (render2D == false) {
		/** E1.5.2: K-D Tree in 3D **/
		// Create viewer
		// TODO: Configure for 3D view
		Box window;
		window.x_min = -10;
		window.x_max = 10;
		window.y_min = -10;
		window.y_max = 10;
		window.z_min = 0;
		window.z_max = 0;
		// TODO: Configure for 3D scene
		pcl::visualization::PCLVisualizer::Ptr viewer = initScene(
			window, 
			25
		);
		/** Creating 3D point data **/
		std::vector<std::vector<float>> points3D = {
			{-6.2, 7.0, 0.0}, {-6.3, 8.4, 1.0}, {-5.2, 7.1, 0.5}, {-5.7, 6.3, 1.5},
			{7.2, 6.1, 3.0}, {8.0, 5.3, 3.5}, {7.2, 7.1, 2.5}, {0.2, -7.1, -4.5},
			{1.7, -6.9, -4.2}, {-1.2, -7.2, -5.5}, {2.2, -8.9, -6.0}
		};
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3D = CreateData3D(
			points3D
		);
		/** E1.5.2: K-D Tree in 3D **/
		KdTree3D* tree3D = new KdTree3D;
		for (int i = 0; i < points3D.size(); i++) {
			tree3D->insert(
				points3D[i],
				i
			);
		}
		int it = 0;
		renderTree3D(
			tree3D->root,
			viewer,
			window, 
			it
		);
		std::cout << "Test Search 3D\n";
		/** E1.3.4: Searching the K-D Tree for nearest neighbours **/
		std::vector<int> nearby = tree3D->search(
			{-6, 7, 0.2}, 
			3.0
		);
		for (int index : nearby) {
			std::cout << index << ",";
		}
		std::cout << std::endl;
		// Time segmentation process
		// auto startTime = std::chrono::steady_clock::now();
		/** E1.5.3: Euclidean Clustering with the K-D Tree **/
		// std::vector<std::vector<int>> clusters = euclideanCluster(
		// 	points3D, 
		// 	tree3D, 
		// 	3.0
		// );
		// auto endTime = std::chrono::steady_clock::now();
		// auto elapsedTime = std::chrono::duration_cast<
		// 	std::chrono::milliseconds
		// >(endTime - startTime);
		// std::cout << "clustering found " << clusters.size()
		// 			<< " and took " << elapsedTime.count() << " milliseconds\n";
		// // Render clusters
		// int clusterId = 0;
		// std::vector<Color> colors = {
		// 	Color(1, 0, 0), 
		// 	Color(0, 1, 0), 
		// 	Color(0, 0, 1)
		// };
		// for (std::vector<int> cluster : clusters) {
		// 	pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(
		// 		new pcl::PointCloud<pcl::PointXYZ>()
		// 	);
		// 	for (int indice : cluster) {
		// 		clusterCloud->points.push_back(
		// 			pcl::PointXYZ(
		// 				points[indice][0],
		// 				points[indice][1],
		// 				0
		// 			)
		// 		);
		// 	}
		// 	renderPointCloud(
		// 		viewer, 
		// 		clusterCloud,
		// 		"cluster" + std::to_string(clusterId),
		// 		colors[clusterId % 3]
		// 	);
		// 	++clusterId;
		// }
		// if (clusters.size() == 0) {
		// 	renderPointCloud(
		// 		viewer,
		// 		cloud,
		// 		"data"
		// 	);
		// }
		while (!viewer->wasStopped()) {
			viewer->spinOnce();
		}
	}
}
