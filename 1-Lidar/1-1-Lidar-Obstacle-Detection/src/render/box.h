/* ------------------------------------------------------------------------------
  * Project "1.1: LiDAR Obstacle Detection"
  * Authors     : Aaron Brown et al.
  *
  * Modified by : Jonathan Logan Moran (jonathan.moran107@gmail.com).
  *
  * Purpose of this file: Defines the structs used to form the 3D scene objects
  * 		e.g., cars and highway lanes. 
  * ----------------------------------------------------------------------------
  */

#ifndef BOX_H
#define BOX_H


#include <Eigen/Geometry> 


/* `BoxQ` struct; 3D cuboid with quaternion rotation.
 *
 * @struct	BoxQ
 * @brief	Cubiod with Quaternion angle.
 * @field	bboxTransform	3D vector defining the transform.
 * @field	bboxQuaternion	Quaternion rotation of the 3D box.
 * @field	cube_length		Length of the cuboid.
 * @field	cube_width		Width of the cuboid.
 * @field	cube_height		Height of the cuboid.
 */
struct BoxQ
{
	Eigen::Vector3f bboxTransform;
	Eigen::Quaternionf bboxQuaternion;
	float cube_length;
    float cube_width;
    float cube_height;
};


/* `Box` struct; 3D cuboid.
 *
 * @struct	Box
 * @brief	Cubiod defined in 3D space.
 * @field	x_min	Minimum coordinate value along $x$-axis.
 * @field	x_max   Maximum coordinate value along $x$-axis.
 * @field	y_min	Minimum coordinate value along $y$-axis.
 * @field	y_max	Maximum coordinate value along $y$-axis.
 * @field	z_min	Minimum coordinate value along $z$-axis.
 * @field	z_max	Maximum coordinate value along $z$-axis.
 */
struct Box
{
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;
};


#endif