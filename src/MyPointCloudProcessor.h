//
// Created by Winfried Auner
//

#ifndef PLAYBACK_MYPOINTCLOUDPROCESSOR_H
#define PLAYBACK_MYPOINTCLOUDPROCESSOR_H

#include "processPointClouds.h"
#include "balancedKdtree.h"
#include <unordered_set>
#include <memory>
#include <random>


template<typename PointT>
class MyPointCloudProcessor : public ProcessPointClouds<PointT> {
private:
    // ransac algorithm implementation to be called from SegmentPlane
    std::unordered_set<int> _ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol) {
        // Time ransac implementation
        auto startTime = std::chrono::steady_clock::now();

        // set up random generator to generate indices for point selection
        std::unordered_set<int> inliersResult;
        std::random_device randomDevice;
        std::default_random_engine e1(randomDevice());
        std::uniform_int_distribution<int> uniform_dist(0, cloud->size() - 1);

        for (int i = 0; i < maxIterations; i++) {
            // chose three distinct random points for possible plane model
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
            // switch code style to keep things readable
            float &x1 = p1.x, &y1 = p1.y, &z1 = p1.z;
            float &x2 = p2.x, &y2 = p2.y, &z2 = p2.z;
            float &x3 = p3.x, &y3 = p3.y, &z3 = p3.z;

            // a plane between two vectors v1[p1->p2] and v2[p1->p3]
            // the cross product of v1 x v2 is the normal vector v3 <v3i,v3j,v3k>
            float v3i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
            float v3j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
            float v3k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);

            // plane formula Ax + By + Cz + D = 0
            float &A = v3i, &B = v3j, &C = v3k;
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
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "ransac took " << elapsedTime.count() << " milliseconds" << std::endl;
        return std::move(inliersResult);
    };

    // cluster algorithm implementation to be called from Clustering
    std::vector<std::vector<int>>
    _euclideanCluster(const std::vector<std::vector<float>>& points, std::shared_ptr<KdTree> tree, float distanceTol) {
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
    };

    void _findClusterPoints(const std::vector<std::vector<float>>& points, int pointIdx, std::vector<int>& cluster,
                            std::unordered_set<int>& processedPoints, std::shared_ptr<KdTree> tree, float distanceTol) {
        // mark point as processed
        processedPoints.insert(pointIdx);
        cluster.push_back(pointIdx);
        std::vector<int> nearestNeighborIds = tree->search(points.at(pointIdx), distanceTol);
        for (int neighborId : nearestNeighborIds) {
            if (processedPoints.find(neighborId) == processedPoints.end()) {
                _findClusterPoints(points, neighborId, cluster, processedPoints, tree, distanceTol);
            }
        }
    };

    std::shared_ptr<KdTree> _createKdtree(typename pcl::PointCloud<PointT>::Ptr cloud) {
        auto kdTreePtr = std::make_shared<KdTree>();
        // naiive tree implementation, not balanced
        for (int id = 0; id < cloud->points.size(); id++) {
            PointT& point = cloud->points.at(id);
            // create a vector from point's array and then add it together with its id into the tree
            kdTreePtr->insert(std::vector<float>(std::begin(point.data_c), std::end(point.data_c)), id);
        }
        return kdTreePtr;
    };

public:

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
    SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold) override {
        // Time segmentation process
        auto startTime = std::chrono::steady_clock::now();

        // find inliers with ransac
        std::unordered_set<int> inliers = _ransac(cloud, maxIterations, distanceThreshold);
        // transfer to other data type/container for SeparateClouds
        pcl::PointIndices::Ptr inliersIndices {new pcl::PointIndices()};
        inliersIndices->indices.reserve(inliers.size());
        for (int inlier : inliers) {
            inliersIndices->indices.push_back(inlier);
        }

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

        return this->SeparateClouds(inliersIndices, cloud);
    };

    // TODO: add override
    std::vector<typename pcl::PointCloud<PointT>::Ptr>
    Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize) override {
        // Time clustering process
        auto startTime = std::chrono::steady_clock::now();

        // create a balanced KDTree from the point cloud
        std::shared_ptr<KdTree> kdTreePtr = std::make_shared<KdTree>(cloud);
        // TODO: check if everything works in 3d, maybe check dimensions in case of segfault

        // extract points to vector for processing
        std::vector<std::vector<float>> points;
        for (auto point : cloud->points) {
            points.emplace_back(std::begin(point.data_c), std::end(point.data_c));
        }
        std::vector<std::vector<int>> clusters = _euclideanCluster(points, kdTreePtr, clusterTolerance);
        // create point clouds for return value based on clusters
        std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters;

        for (std::vector<int> cluster : clusters) {
            // create a point cloud for every cluster and add the corresponding points to it
            typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>());
            cloudCluster->points.reserve(cluster.size());
            for (int pointIndex : cluster) {
                cloudCluster->points.emplace_back(cloud->points.at(pointIndex));
            }
            // add the newly created cluster point cloud to the return container
            cloudClusters.push_back(cloudCluster);
        }
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size()
                  << " clusters" << std::endl;
        return cloudClusters;
    };

};


#endif //PLAYBACK_MYPOINTCLOUDPROCESSOR_H
