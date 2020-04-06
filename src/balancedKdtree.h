// my custom balanced kd tree

#include "pcl/common/common.h"
#include <algorithm>

// does not take ownership or make copies of the underlying datastructure
// just references to the memory of the original pointcloud
// to be as fast as possible
struct Node {
    pcl::PointXYZI* point;
    Node* left;
    Node* right;
    // maybe add depth member

    Node(pcl::PointXYZI* point) : point(point), left(nullptr), right(nullptr) {};

};

class KdTree {
    Node* _root;

    // to be called from constructor
    void _insertPoints(std::vector<pcl::PointXYZI*>& points, int depth, Node*& currentNode);
    void _searchClusterPoints(std::vector<pcl::PointXYZI*>& clusterPoints, pcl::PointXYZI* queryPoint, const float& distanceTol, Node* nodeToCheck, int depth);
    bool _checkIfInBox(pcl::PointXYZI* queryPoint, Node* comparisonNode, float distanceTol) const;

public:
    KdTree() : _root(nullptr) {};
    // custom constructor to create a balanced KDtree from pointcloud
    KdTree(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    std::vector<pcl::PointXYZI*> findClusterPoints(pcl::PointXYZI* queryPoint, float distanceTol);
};