//
// Created by Winfried Auner
//

#include <random>
#include "MyPointCloudProcessor.h"

// performs the _ransac algorithm and returns the indices of the point that lie in the plane
template<typename PointT>
std::unordered_set<int>
MyPointCloudProcessor<PointT>::_ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol){
    // set up random generator
    std::unordered_set<int> inliersResult;
    std::random_device randomDevice;
    std::default_random_engine e1(randomDevice());
    std::uniform_int_distribution<int> uniform_dist(0, cloud->size() - 1);

    for (int i = 0; i < maxIterations; i++) {
        // chose three random points for possible plane model
        std::unordered_set<int> inliersIteration;
        while (inliersIteration.size() < 3) {
            inliersIteration.insert(uniform_dist(e1));
        }

        // fit line between the three points
        // get a reference to each of the randomly selected points
        auto indicesItr = inliersIteration.begin();
        PointT& p1 = cloud->points.at(*indicesItr);
        indicesItr++;
        PointT& p2 = cloud->points.at(*indicesItr);
        indicesItr++;
        PointT& p3 = cloud->points.at(*indicesItr);

        // aliases to make formula more readable
        float& x1 = p1.x, y1 = p1.y, z1 = p1.z;
        float& x2 = p2.x, y2 = p2.y, z2 = p2.z;
        float& x3 = p3.x, y3 = p3.y, z3 = p3.z;

        // a plane between two vectors v1[p1->p2] and v2[p1->p3]
        // the cross product of v1 x v2 is the normal vector v3 <v3i,v3j,v3k>
        float v3i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        float v3j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        float v3k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);

        // plane formula Ax + By + Cz + D = 0
        float& A = v3i, B = v3j, C = v3k;
        float D = -(v3i * x1 + v3j * y1 + v3k * z1);

        // loop over points in cloud and check whether they are inliers
        for (int idx = 0; idx < cloud->points.size(); idx++) {
            // skip selected points
            if (inliersIteration.count(idx)) {
                continue;
            }
            // calculate distance between point and model line
            float& x4 = cloud->points.at(idx).x;
            float& y4 = cloud->points.at(idx).y;
            float& z4 = cloud->points.at(idx).z;

            float distance = fabs(A * x4 + B * y4 + C * z4 + D) / sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2));
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


// Segments the cloud into a cloud containing the road points and one containing everything else
// by using my own _ransac implementation
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
MyPointCloudProcessor<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
                                            float distanceThreshold) {
    // find inliers with ransac
    std::unordered_set<int> inliers = _ransac(cloud, maxIterations, distanceThreshold);
    // transfer to other data type/container for SeparateClouds
    pcl::PointIndices::Ptr inliersIndices;
    for (int inlier : inliers) {
        inliersIndices->indices.push_back(inlier);
    }

    return SeparateClouds(inliers, cloud);
}

// recursive helper to perform clustering by traversing a KDTree, is called from _euclideanCluster
template<typename PointT>
void MyPointCloudProcessor<PointT>::_findClusterPoints(const std::vector<std::vector<float>>& points, int pointIdx,
                                                       std::vector<int>& cluster,
                                                       std::unordered_set<int>& processedPoints, KdTree* tree,
                                                       float distanceTol){
    // mark point as processed
    processedPoints.insert(pointIdx);
    cluster.push_back(pointIdx);
    std::vector<int> nearestNeighborIds = tree->search(points.at(pointIdx), distanceTol);
    for (int neighborId : nearestNeighborIds) {
        if (processedPoints.find(neighborId) == processedPoints.end()) {
            _findClusterPoints(points, neighborId, cluster, processedPoints, tree, distanceTol);
        }
    }
}

// performs euclidean clustering and returns the point indices from the original cloud
// for each cluster
template<typename PointT>
std::vector<std::vector<int>>
MyPointCloudProcessor<PointT>::_euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree,
                                                 float distanceTol) {

    std::vector<std::vector<int>> clusters;
    std::unordered_set<int> processedPoints{};
    for (int pointIdx = 0; pointIdx < points.size(); pointIdx++) {
        // if point has not yet been processed
        if (processedPoints.find(pointIdx) == processedPoints.end()) {
            std::vector<int> cluster;
            _findClusterPoints(points, pointIdx, cluster, processedPoints, tree, distanceTol);
            clusters.push_back(std::move(cluster));
        }
    }
    return clusters;
}

template<typename PointT>
std::shared_ptr<KdTree> MyPointCloudProcessor<PointT>::_createKdtree(typename pcl::PointCloud<PointT>::Ptr cloud) {
    auto kdTreePtr = std::make_shared<KdTree>();
    // naiive tree implementation, not balanced
    for (int id = 0; id < cloud->points.size(); id++) {
        PointT& point = cloud->points.at(id);
        kdTreePtr->insert(std::vector<float>(point.data), id);
    }
    return kdTreePtr;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
MyPointCloudProcessor<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize) {
    // TODO: insert all points into KDTree, write function for creating a balanced kdtree
    std::shared_ptr<KdTree> kdTreePtr = _createKdtree(cloud);
    // TODO: check if everything works in 3d, maybe check dimensions in case of segfault

    // extract points to vector for processing
    std::vector<std::vector<float>> points;
    for (auto point : cloud->points) {
        points.emplace_back(std::vector<float>(point.data));
    }
    std::vector<std::vector<int>> clusters = _euclideanCluster(points, kdTreePtr, clusterTolerance);
    // create point clouds for return value based on clusters
    std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters;

    for (std::vector<int> cluster : clusters) {
        // create a point cloud for every cluster and add the corresponding points to it
        typename pcl::PointCloud<PointT>::Ptr cloudCluster = new pcl::PointCloud<PointT>();
        cloudCluster->points.reserve(cluster.size());
        for (int pointIndex : cluster) {
            cloudCluster->points.emplace_back(cloud->points.at(pointIndex));
        }
        // add the newly created cluster point cloud to the return container
        cloudClusters.push_back(cloudCluster);
    }
    return cloudClusters;
}
