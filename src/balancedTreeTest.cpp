//
// Created by mimimint on 4/6/20.
//
#include "pcl/common/common.h"
#include "balancedKdtree.h"
#include <random>

pcl::PointCloud<pcl::PointXYZI>::Ptr createData() {
    // create random data
    std::random_device randomDevice;
    std::default_random_engine e1(randomDevice());
    std::uniform_int_distribution<int> uniform_dist(0, 100);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    // create 10 test points
    for (int i = 0; i<10; i++) {
        pcl::PointXYZI point;
        point.x = uniform_dist(e1);
        point.y = uniform_dist(e1);
        point.z = uniform_dist(e1);
        point.intensity = uniform_dist(e1);

        cloud->points.push_back(point);
    }
    return cloud;
}

int main() {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = createData();
    KdTree tree(cloud);
}


