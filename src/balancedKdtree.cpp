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
        return pointA->data[depth % 3] < pointB->data[depth % 3];
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
}

// public interface, returns the pointer to the points that belong to a cluster
std::vector<pcl::PointXYZI*> KdTree::findClusterPoints(pcl::PointXYZI* queryPoint, float distanceTol) {
    std::vector<pcl::PointXYZI*> clusterPoints;
    _searchClusterPoints(clusterPoints, queryPoint, distanceTol, _root,0);

    return clusterPoints;
}

// traverses the tree and looks for points close the queryPoint
void
KdTree::_searchClusterPoints(std::vector<pcl::PointXYZI*>& clusterPoints, pcl::PointXYZI* queryPoint, const float& distanceTol,
                             Node* nodeToCheck, int depth) {
    // return if end of tree is reached
    if (nodeToCheck != nullptr) {
        // check if points are close within cube
        if (_checkIfInBox(queryPoint, nodeToCheck, distanceTol)) {
            clusterPoints.push_back(nodeToCheck->point);
        }
        float& splitValQuery = queryPoint->data[depth % 3];
        float& splitValNode = nodeToCheck->point->data[depth % 3];

        // test which sub spaces need to be explored
        // if the bounding box around the target box overlaps with a subspace, explore it
        if (splitValQuery - distanceTol <= splitValNode) {
            _searchClusterPoints(clusterPoints, queryPoint, distanceTol, nodeToCheck->left, depth + 1);
        }
        if (splitValQuery + distanceTol >= splitValNode) {
            _searchClusterPoints(clusterPoints, queryPoint, distanceTol, nodeToCheck->right, depth + 1);
        }
    }
}

bool KdTree::_checkIfInBox(pcl::PointXYZI* queryPoint, Node* comparisonNode, float distanceTol) const {
    bool inX = queryPoint->x + distanceTol >= comparisonNode->point->x &&
               queryPoint->x - distanceTol <= comparisonNode->point->x;
    bool inY = queryPoint->y + distanceTol >= comparisonNode->point->y &&
               queryPoint->y - distanceTol <= comparisonNode->point->y;
    bool inZ = queryPoint->z + distanceTol >= comparisonNode->point->z &&
               queryPoint->z - distanceTol <= comparisonNode->point->z;

    return inX && inY && inZ;
};
