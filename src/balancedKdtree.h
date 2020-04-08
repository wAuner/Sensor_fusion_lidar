// my custom balanced kd tree

#include "pcl/common/common.h"
#include <algorithm>

// does not take ownership or make copies of the underlying datastructure
// just references to the memory of the original pointcloud
// to be as fast as possible
template <typename PointT>
struct Node {
    // does not take ownership or change the point
    PointT* point;
    // make sure all tree nodes are cleaned up
    std::shared_ptr<Node> left;
    std::shared_ptr<Node> right;

    Node(PointT* point) : point(point), left(nullptr), right(nullptr) {};

};

template <typename PointT>
class KdTree {
    std::shared_ptr<Node<PointT>> _root;

    // to be called from constructor
    void _insertPoints(std::vector<PointT*>& points, int depth, std::shared_ptr<Node<PointT>>& currentNode) {
        // lambda func to allow sorting of points based on alternating features depending on tree depth
        auto comparePoints = [depth](const PointT* pointA, const PointT* pointB)-> bool{
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
        PointT* pointToSplitOn = points.at(medianPos);

        // find median
        // split left right
        // handle base cases of recursion
        if (currentNode == nullptr) {
            currentNode = std::make_shared<Node<PointT>>(pointToSplitOn);
            // base case if only two points are left
            if (points.size() == 2) {
                currentNode->right = std::make_shared<Node<PointT>>(points.at(1));
                return;
            } else if (points.size() == 1) {
                return;
            }
            // everything smaller than the median goes into left branch, the rest in the right
            // median is the splitting point that separates the space
            // iterator is half open range[begin, end)
            std::vector<PointT*> pointsLeft(points.begin(), points.begin() + medianPos);
            // skip median point to avoid inserting it twice
            std::vector<PointT*> pointsRight(points.begin() + medianPos + 1, points.end());
            _insertPoints(pointsLeft, depth + 1, currentNode->left);
            _insertPoints(pointsRight, depth + 1, currentNode->right);
        }
    }

    // traverses the tree and looks for points close the queryPoint
    void _searchClusterPoints(std::vector<PointT*>& clusterPoints, PointT* queryPoint, const float& distanceTol,
                                         std::shared_ptr<Node<PointT>>& nodeToCheck, int depth) {
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


    bool _checkIfInBox(PointT* queryPoint, std::shared_ptr<Node<PointT>>& comparisonNode, float distanceTol) const {
        bool inX = queryPoint->x + distanceTol >= comparisonNode->point->x &&
                   queryPoint->x - distanceTol <= comparisonNode->point->x;
        bool inY = queryPoint->y + distanceTol >= comparisonNode->point->y &&
                   queryPoint->y - distanceTol <= comparisonNode->point->y;
        bool inZ = queryPoint->z + distanceTol >= comparisonNode->point->z &&
                   queryPoint->z - distanceTol <= comparisonNode->point->z;

        return inX && inY && inZ;
    };

public:
    typename pcl::PointCloud<PointT>::Ptr _cloudPtr;
    KdTree() : _root(nullptr) {};
    // custom constructor to create a balanced KDtree from pointcloud
    KdTree(typename pcl::PointCloud<PointT>::Ptr cloud) : _root(nullptr), _cloudPtr(cloud) {
        // use pointers to make things super fast
        std::vector<PointT*> points;
        points.reserve(cloud->points.size());
        for (auto& point : cloud->points) {
            points.push_back(&point);
        }
        _insertPoints(points, 0, _root);
    }

    // public interface, returns the pointer to the points that are within distanceTol of the queryPoint
    std::vector<PointT*> search(PointT* queryPoint, float distanceTol) {
        std::vector<PointT*> clusterPoints;
        _searchClusterPoints(clusterPoints, queryPoint, distanceTol, _root,0);

        return clusterPoints;
    };
};

