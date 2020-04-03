/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <memory.h>

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Done:: Create lidar sensor
    std::shared_ptr<Lidar> lidar = std::make_shared<Lidar>(cars, 0);

    // Done:: Create point processor
    auto pointCloud = lidar->scan();
    //renderRays(viewer, lidar->position, pointCloud);
    renderPointCloud(viewer, pointCloud, "utz");

    auto processPointClouds = std::make_shared<ProcessPointClouds<pcl::PointXYZ>>();
    // segment cloud into objects and road
    auto segmentedClouds = processPointClouds->SegmentPlane(pointCloud, 100, 0.2);
    //renderPointCloud(viewer, segmentedClouds.first, "obstacleCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentedClouds.second, "planeCloud", Color(0,1,0));

    // find clusters in obstacle cloud
    auto cloudClusters = processPointClouds->Clustering(segmentedClouds.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    // render clusters
    // warning: rendering limited to three clusters because of
    // # defined colors
    for (auto& cluster : cloudClusters) {
        std::cout << "cluster size of cloud #" << clusterId << ": ";
        // stupid print function...
        processPointClouds->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors.at(clusterId));
        Box box = processPointClouds->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        clusterId++;
    }
  
}

// performs downsampling, segmentation (road/obstacles) and clustering of the input point cloud
// renders either downsampled processing result or the resulting bounding boxes over the original cloud
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, std::shared_ptr<ProcessPointClouds<pcl::PointXYZI>> pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud, const bool renderFullCloud) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    //renderPointCloud(viewer,inputCloud,"inputCloud");

    // define downsampling parameters and region of interest
    float voxelSize = 0.1; // 1cm grid size
    Eigen::Vector4f minPoint {-20, -10, -2, 1};
    Eigen::Vector4f maxPoint {20, 10, 2, 1};
    // downsampling of the cloud data to increase performance
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, voxelSize, minPoint,
                                                                                    maxPoint);
    //renderPointCloud(viewer, filterCloud, "filterCloud");
    // segment cloud into road and obstacles
    auto segmentedClouds = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);

    // render either full cloud and later add bounding boxes or render downsampled road cloud first and clusters
    // with bounding boxes later
    if (renderFullCloud) {
        renderPointCloud(viewer, inputCloud, "fullCloud");
    } else {
        // render road cloud points in green
        renderPointCloud(viewer, segmentedClouds.second, "roadCloud", Color(0, 1, 0));
    }

    // find objects as clusters in obstacle cloud
    auto obstacleClouds = pointProcessorI->Clustering(segmentedClouds.first, .5, 10, 3000);
    // just some colors to render objects differently; colors don't indicate classification
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    int objIndex = 0;
    for (auto& obstacleCloud : obstacleClouds) {
        // render clusters in downsampled view
        if (!renderFullCloud) {
            renderPointCloud(viewer, obstacleCloud, "object#" + std::to_string(objIndex),
                             colors.at(objIndex % colors.size()));
        }
        Box box = pointProcessorI->BoundingBox(obstacleCloud);
        renderBox(viewer, box, objIndex, Color(1,1,1));
        objIndex++;
    }

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    // set this to true to perform algorithms on the downsampled cloud
    // but visualize the results using the HD-Cloud
    const bool renderFullCloud = true;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    auto pointProcessorI = std::make_shared<ProcessPointClouds<pcl::PointXYZI>>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // load point cloud and apply processing pipeline
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        // perform processing and rendering
        cityBlock(viewer, pointProcessorI, inputCloudI, renderFullCloud);

        streamIterator++;
        if (streamIterator == stream.end()) {
            streamIterator = stream.begin();
        }

        viewer->spinOnce ();
    }
}