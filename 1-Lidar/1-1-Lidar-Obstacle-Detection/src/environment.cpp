/* ----------------------------------------------------------------------------
  * Project "1.1: LiDAR Obstacle Detection"
  * Authors     : Aaron Brown et al.
  *
  * Modified by : Jonathan Logan Moran (jonathan.moran107@gmail.com).
  *
  * Purpose of this file: Entry-point to the simple 3D highway programme using
  *         Point Cloud Library (PCL). Defined here are several functions to
  *         render a 3D environment and its scene objects, as well as functions
  *         to control the camera view-point and instantiate a simulated LiDAR
  *         sensor. This corresponds to Exercises 1.1.0-8 of Course 1: Lidar in
  *         the Sensor Fusion Nanodegree offered by Udacity.
  * ---------------------------------------------------------------------------
  */

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


/* Initialises the 3D "highway" environment. 
 * 
 * @brief   Creates the vector of objects for the 3D highway environment.
 * @param   renderScene     If `true`, the objects created will be displayed.
 * @param   viewer          The PCL window to render the objects onto.
 * @returns Vector of `Car` objects to be rendered in the 3D environment.
 */
std::vector<Car> initHighway(
    bool renderScene, 
    pcl::visualization::PCLVisualizer::Ptr& viewer
) {
    Car egoCar(
        Vect3(0, 0, 0), 
        Vect3(4, 2, 2), 
        Color(0, 1, 0), 
        "egoCar"
    );
    Car car1(
        Vect3(15, 0, 0), 
        Vect3(4, 2, 2), 
        Color(0, 0, 1), 
        "car1"
    );
    Car car2(
        Vect3(8, -4, 0), 
        Vect3(4, 2, 2), 
        Color(0, 0, 1), 
        "car2"
    );
    Car car3(
        Vect3(-12, 4, 0), 
        Vect3(4, 2, 2), 
        Color(0, 0, 1), 
        "car3"
    );
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);
    if (renderScene) {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }
    return cars;
}


/* Performs the 3D highway environment simulation.
 *
 * Here, the 3D highway scene is created. A LiDAR sensor object is initialised
 * and configured. The LiDAR sensor performs a "scan" of the environment, then
 * the resulting point cloud measurements (and rays emitted from the sensor)
 * are visualised onto the environment.
 *
 * @brief   Creates the 3D environment and controls the LiDAR sensor object.
 * @param   viewer          The PCL window to render the objects onto.
 */
void simpleHighway(
    pcl::visualization::PCLVisualizer::Ptr& viewer
) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    /** E1.1.0: Create 3D highway scene. **/
    // RENDER OPTIONS
    // (1) Set `renderScene` to `false`: removes the highway lanes and vehicle objects.
    bool renderScene = true;
    // Displaying the "simple highway" scene objects
    std::vector<Car> cars = initHighway(
        renderScene, 
        viewer
    );
    /** E1.1.1: Create LiDAR sensor object. **/
    // SENSOR SPECIFICATIONS
    float groundSlope = 0.0;                        // Radians; angle of departure between $x$-$y$ plane.
    // Instantiating a simulated LiDAR sensor object
    Lidar *lidar = new Lidar(
        cars,
        groundSlope
    );
    /** E1.1.2: Project the LiDAR rays onto the scene. **/
    // Generating a new Lidar sensor scan
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = lidar->scan();
    // RENDER OPTIONS
    // (2) Comment out `renderRays`: removes the simulated LiDAR laser beams.
    // Projecting the LiDAR sensor rays onto the scene
    renderRays(
        viewer,
        lidar->position,
        pointCloud
    );
    /** E1.1.3: Display the detected points. **/
    // Displaying the PCD measurements
    renderPointCloud(
        viewer,
        pointCloud,
        "pointCloud"
    );
    // TODO:: Create point processor
}


/* Initialises the camera view within the 3D environment.
 *
 * @brief   Sets the view-point at which the scene is viewed. 
 * @param   setAngle    Pre-defined viewing angle; can choose one `CameraAngle` 
 *                      from the following: [`XY`, `TopDown`, `Side`, `FPS`].
 * @param   viewer      The PCL window to control the view-point within.
 */
void initCamera(
    CameraAngle setAngle, 
    pcl::visualization::PCLVisualizer::Ptr& viewer
) {
    viewer->setBackgroundColor(0, 0, 0); 
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    switch (setAngle) {
        case XY : 
            viewer->setCameraPosition(-distance, 
                                      -distance, 
                                      distance, 
                                      1, 
                                      1, 
                                      0
            ); 
            break;
        case TopDown : 
            viewer->setCameraPosition(0, 
                                      0, 
                                      distance, 
                                      1, 
                                      0, 
                                      1
            ); 
            break;
        case Side : 
            viewer->setCameraPosition(0, 
                                      -distance, 
                                      0, 
                                      0, 
                                      0, 
                                      1
            ); 
            break;
        case FPS : 
            viewer->setCameraPosition(-10, 
                                      0, 
                                      0, 
                                      0, 
                                      0, 
                                      1
            );
    }
    if (setAngle!= FPS) {
        viewer->addCoordinateSystem(1.0);
    }
}


/* Entry-point for the simple highway simulation programme.
 *
 * @brief Initialises the 3D viewer and runs the simple highway simulation. 
 */
int main(
    int argc, 
    char** argv
) {
    std::cout << "starting enviroment" << std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer")
    );
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    simpleHighway(viewer);
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}