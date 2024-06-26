/* ----------------------------------------------------------------------------
  * Project "1.1: LiDAR Obstacle Detection"
  * Authors     : Aaron Brown et al.
  *
  * Modified by : Jonathan Logan Moran (jonathan.moran107@gmail.com).
  *
  * Purpose of this file: Defines the Lidar sensor and sensor Ray structs.
  * ---------------------------------------------------------------------------
  */

#ifndef LIDAR_H
#define LIDAR_H


#include "../render/render.h"
#include <ctime>
#include <chrono>


const double pi = 3.1415;


/* Lidar `Ray` struct.
 *
 * @struct	Ray
 * @brief	Simulated ray generated from a LiDAR sensor.
 * @field	origin			Starting position in 3D from where the ray is cast. 
 * @field	resolution		Magnitude of distance moved by ray at each step.
 * @field	direction		Direction in 3D of which the ray travels. 
 * @field	castPosition	Current position in 3D of the ray.  
 * @field	castDistance	Distance from ray's current point to origin.
 * @brief	Projects the ray instance into the 3D environment.
 */
struct Ray {
	Vect3 origin;
	double resolution;
	Vect3 direction;
	Vect3 castPosition;
	double castDistance;
	Ray(Vect3 setOrigin, 
		double horizontalAngle, 
		double verticalAngle, 
		double setResolution
	) : origin(setOrigin), 
		resolution(setResolution),
		direction(
			resolution * cos(verticalAngle) * cos(horizontalAngle), 
			resolution * cos(verticalAngle) * sin(horizontalAngle),
			resolution * sin(verticalAngle)
		),
		castPosition(origin),
		castDistance(0) {}
	
	/* Simulates LiDAR sensor ray casting.
	 * 
	 * Populates the given `cloud` object with ray traces projected from
	 * the origin of the LiDAR sensor onto "reflected" objects. Some amount of
	 * measurement uncertainty may be considered for non-zero `sderr` values.  
	 *
	 * @brief 	Simulates ray-casting for the given `Ray` instance.
	 * @param	cars		Vector of `Car` objects in the environment.
	 * @param	minDistance	Minimum distance (m); points closer will be deleted.
	 * @param   maxDistance Maximum distance (m); points further will be deleted.
	 * @param   cloud		Point cloud object to populate with rays.
	 * @param	slopeAngle	Angle (rad) of departure from $x$-$y$ plane.
	 * @param	sderr		Standard deviation of error; measurement uncertainty.
	 */
	void rayCast(
		const std::vector<Car>& cars, 
		double minDistance, 
		double maxDistance, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
		double slopeAngle, 
		double sderr
	) {
		// reset ray
		castPosition = origin;
		castDistance = 0;
		bool collision = false;
		while (!collision && castDistance < maxDistance) {
			castPosition = castPosition + direction;
			castDistance += resolution;
			// check if there is any collisions with ground slope
			collision = (castPosition.z <= castPosition.x * tan(slopeAngle));
			// check if there is any collisions with cars
			if (!collision && castDistance < maxDistance) {
				for (Car car : cars) {
					collision |= car.checkCollision(castPosition);
					if (collision) {
						break;
					}
				}
			}
		}
		if (castDistance >= minDistance
			&& castDistance <= maxDistance
		) {
			// add noise based on standard deviation error
			double rx = ((double) rand() / (RAND_MAX));
			double ry = ((double) rand() / (RAND_MAX));
			double rz = ((double) rand() / (RAND_MAX));
			cloud->points.push_back(
				pcl::PointXYZ(
					castPosition.x + rx * sderr, 
					castPosition.y + ry * sderr, 
					castPosition.z + rz * sderr
				)
			);
		}
	}
};


/* Lidar sensor struct. 
 *
 * @struct	Lidar
 * @brief	Simulated Lidar sensor object.
 * @field	rays			Vector containing all ray projections.
 * @field	cloud			Vector containing all LiDAR points.
 * @field	cars			Vector containing all `Car` instances.
 * @field	position		Origin in 3D of the LiDAR sensor instance.
 * @field	groundSlope		Angle (rad) of ray projected from $x$-$y$ plane.
 * @field	minDistance		Minimum distance (m); points closer will be deleted.
 * @field	maxDistance		Maximum distance (m); points further will be deleted.
 * @field	resoultion		Magnitude of distance moved by ray at each step.
 * @field	sderr			Standard deviation of error; measurement uncertainty.
 * @func	pcl::PointCloud<pcl::PointXYZ>::Ptr		scan
 * @brief	Generates a point cloud scan of the environment.
 * */
struct Lidar {
	std::vector<Ray> rays;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	std::vector<Car> cars;
	Vect3 position;
	double groundSlope;
	double minDistance;
	double maxDistance;
	double resoultion;
	double sderr;

	Lidar(
		std::vector<Car> setCars, 
		double setGroundSlope
	) : cloud(new pcl::PointCloud<pcl::PointXYZ>()),
		position(0, 0, 2.6) {
		resoultion = 0.2;
		cars = setCars;
		groundSlope = setGroundSlope;
		/*** E1.1.4-7: Modify the LiDAR sensor parameters. ***/
        /** E1.1.4: Increase the vertical scanning resolution. **/
		// Modifying the number of vertical beams to increase resolution
        int numLayers = 8;                      	// Number of vertically-stacked diodes
		/** E1.1.5: Increase the horizontal scanning resolution. **/
        // Modifying the angular "spacing" to improve resolution
        double horizontalAngleInc = pi / 64;     	// Radians
		/** E1.1.6: "Remove" the points reflected off the ego-vehicle. **/
        // Discarding all points at distances below `minDistance`
		minDistance = 5;                        	// Distance given in metres (m)
		maxDistance = 50;
		/** E1.1.7: Add "noise" to the sensor measurements. **/
        // Modelling measurement uncertainty with non-zero value
        sderr = 0.0;                            	// Standard deviation of error
		/* Other LiDAR parameters */
		// Steepest vertical angle
		double steepestAngle =  30.0 * (-pi / 180);
		double angleRange = 26.0 * (pi / 180);
		double angleIncrement = angleRange / numLayers;
		for (double angleVertical = steepestAngle; angleVertical < steepestAngle + angleRange; angleVertical += angleIncrement) {
			for (double angle = 0; angle <= 2 * pi; angle += horizontalAngleInc) {
				Ray ray(position,
						angle,
						angleVertical,
						resoultion
				);
				rays.push_back(ray);
			}
		}
	}
	~Lidar() {
		// NOTE: PCL uses `boost` smart pointers for cloud pointer, 
		// So we don't have to worry about manually freeing the memory.
	}

	/* Performs a "scan" of the environment.
	 *
	 * @brief 	Performs a "scan" of the environment.
	 * @returns	The point cloud object.
	 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr scan() {
		cloud->points.clear();
		auto startTime = std::chrono::steady_clock::now();
		for(Ray ray : rays)
			ray.rayCast(cars, 
						minDistance, 
						maxDistance, 
						cloud, 
						groundSlope, 
						sderr
		);
		auto endTime = std::chrono::steady_clock::now();
		auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
			endTime - startTime
		);
		cout << "ray casting took " << elapsedTime.count() << " milliseconds" << endl;
		cloud->width = cloud->points.size();
		cloud->height = 1; // one dimensional unorganized point cloud dataset
		return cloud;
	}
};


#endif