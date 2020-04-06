#include "render/render.h"
#include "environmentHelpers.h"
#include "processPointClouds.h"
#include "MyPointCloudProcessor.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <memory>


int main() {
    std::cout << "starting enviroment" << std::endl;

    // set this to true to perform algorithms on the downsampled cloud
    // but visualize the results using the HD-Cloud
    const bool renderFullCloud = true;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    std::shared_ptr<ProcessPointClouds<pcl::PointXYZI>> pointProcessorI = std::make_shared<MyPointCloudProcessor<pcl::PointXYZI>>();
    //std::shared_ptr<ProcessPointClouds<pcl::PointXYZI>> pointProcessorI = std::make_shared<ProcessPointClouds<pcl::PointXYZI>>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped())
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