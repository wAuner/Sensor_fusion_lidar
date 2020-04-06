//
// Created by mimimint on 4/6/20.
//
#include "balancedKdtree.h"

// custom constructor to create a balanced KDtree from pointcloud
KdTree::KdTree(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) : _root(nullptr) {
    // use pointers to make things super fast
    std::vector<pcl::PointXYZI*> points;
    points.reserve(cloud->points.size());
    for (auto& point : cloud->points) {
        points.push_back(&point);
    }
    _insertPoints(points, 0, _root);
}

void KdTree::_insertPoints(std::vector<pcl::PointXYZI*>& points, int depth, Node*& currentNode) {
    // lambda func to allow sorting of points based on alternating features depending on tree depth
    auto comparePoints = [depth](const pcl::PointXYZI* pointA, const pcl::PointXYZI* pointB)-> bool{
        return pointA->data_c[depth % 3] < pointB->data_c[depth % 3];
    };
    std::sort(points.begin(), points.end(), comparePoints);
    // find median index
    std::size_t medianPos = points.size() / 2;
    // if vector size is even, pick smaller index
    if (points.size() % 2 == 0) {
        medianPos--;
    }
    // median point
    pcl::PointXYZI* pointToSplitOn = points.at(medianPos);

    // TODO: implement inserting recursive
    // find median
    //split left right
    if (currentNode == nullptr) {
        currentNode = new Node(pointToSplitOn);
        // base case if only two points are left
        if (points.size() == 2) {
            currentNode->right = new Node(points.at(1));
            return;
        }
        // iterator is half open range[begin, end)
        std::vector<pcl::PointXYZI*> pointsLeft(points.begin(), points.begin() + medianPos);
        // skip median point to avoid inserting it twice
        std::vector<pcl::PointXYZI*> pointsRight(points.begin() + medianPos + 1, points.end());
        _insertPoints(pointsLeft, depth + 1, currentNode->left);
        _insertPoints(pointsRight, depth + 1, currentNode->right);
    }
};
