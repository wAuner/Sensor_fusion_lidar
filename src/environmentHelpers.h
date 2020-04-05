//
// Created by mimimint on 4/4/20.
//

#ifndef PLAYBACK_ENVIRONMENTHELPERS_H
#define PLAYBACK_ENVIRONMENTHELPERS_H

#include "processPointClouds.h"
#include "render/render.h"
#include "MyPointCloudProcessor.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <memory>

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

// performs downsampling, segmentation (road/obstacles) and clustering of the input point cloud
// renders either downsampled processing result or the resulting bounding boxes over the original cloud
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               std::shared_ptr<ProcessPointClouds<pcl::PointXYZI>> pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud, const bool renderFullCloud) {

    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    //renderPointCloud(viewer,inputCloud,"inputCloud");

    // define downsampling parameters and region of interest
    float voxelSize = 0.5; // 50cm grid size
    Eigen::Vector4f minPoint{-20, -10, -2, 1};
    Eigen::Vector4f maxPoint{20, 10, 2, 1};
    // downsampling of the cloud data to increase performance
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, voxelSize, minPoint,
                                                                                    maxPoint);
    //renderPointCloud(viewer, filterCloud, "filterCloud");
    // segment cloud into road and obstacles
    auto segmentedClouds = pointProcessorI->SegmentPlane(filterCloud, 50, 0.2);

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
        renderBox(viewer, box, objIndex, Color(1, 1, 1));
        objIndex++;
    }

}

#endif //PLAYBACK_ENVIRONMENTHELPERS_H
