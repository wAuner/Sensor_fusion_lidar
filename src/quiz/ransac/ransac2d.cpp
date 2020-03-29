/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <random>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for (int i = -5; i < 5; i++) {
        double rx = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = i + scatter * rx;
        point.y = i + scatter * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while (numOutliers--) {
        double rx = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = 5 * rx;
        point.y = 5 * ry;
        point.z = 0;

        cloud->points.push_back(point);

    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene() {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem(1.0);
    return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) {
    // set up random generator
    std::unordered_set<int> inliersResult;
    std::random_device randomDevice;
    std::default_random_engine e1(randomDevice());
    std::uniform_int_distribution<int> uniform_dist(0, cloud->size() - 1);

    for (int i = 0; i < maxIterations; i++) {
        int randIdx1 = uniform_dist(e1);
        int randIdx2 = uniform_dist(e1);
        while (randIdx2 == randIdx1) {
            randIdx2 = uniform_dist(e1);
        }

        // fit line between the two points
        float A, B, C;
        pcl::PointXYZ& p1 = cloud->points.at(randIdx1);
        pcl::PointXYZ& p2 = cloud->points.at(randIdx2);

        float& x1 = p1.x;
        float& y1 = p1.y;
        float& x2 = p2.x;
        float& y2 = p2.y;

        // parameters for two point form
        A = y1 - y2;
        B = x2 - x1;
        C = x1 * y2 - x2 * y1;

        // count inliers of current model
        // start with sample points
        std::unordered_set<int> inliersIteration;
        inliersIteration.insert(randIdx1);
        inliersIteration.insert(randIdx2);

        // loop over points in cloud and check whether they are inliers
        for (int idx = 0; idx < cloud->points.size(); idx++) {
            // skip selected points
            if (idx == randIdx1 || idx == randIdx2) {
                continue;
            }
            // calculate distance between point and model line
            float& x3 = cloud->points.at(idx).x;
            float& y3 = cloud->points.at(idx).y;

            float distance = fabs(A * x3 + B * y3 + C) / sqrt(pow(A, 2) + pow(B, 2));
            // add inliers
            if (distance <= distanceTol) {
                inliersIteration.insert(idx);
            }
        }
        // if model has more inliers, choose the model
        if (inliersIteration.size() > inliersResult.size()) {
            inliersResult = std::move(inliersIteration);
        }
    }
    return inliersResult;
}

int main() {

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();


    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for (int index = 0; index < cloud->points.size(); index++) {
        pcl::PointXYZ point = cloud->points[index];
        if (inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }


    // Render 2D point cloud with inliers and outliers
    if (inliers.size()) {
        renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
        renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
    } else {
        renderPointCloud(viewer, cloud, "data");
    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }

}
