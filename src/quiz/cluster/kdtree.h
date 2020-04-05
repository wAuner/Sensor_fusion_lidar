/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node {
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(const std::vector<float>& arr, int setId)
            : point(arr), id(setId), left(nullptr), right(nullptr) {}
};

struct KdTree {
    Node* root;

    KdTree()
            : root(nullptr) {}

    void insert(const std::vector<float>& point, int id) {
        // point[0] is x, point[1] is y
        // the function should create a new node and place correctly with in the root
        int treeDepth = 0;
        Node** currentNode = &root;
        // traverse the tree until you hit a leaf, then insert
        while (true) {
            if (*currentNode == nullptr) {
                *currentNode = new Node(point, id);
                break;
            } else {
                // serves as a switch on which feature to split
                // needs to changed for 2d / 3d case
                int splitModulo = 2;
                float x_new = point.at(0);
                float y_new = point.at(1);
                float x_node = (*currentNode)->point.at(0);
                float y_node = (*currentNode)->point.at(1);
                // in case of 3d add z coordinate and change feature split switch
                // to alternate between all 3 spacial dimensions
                float z_new, z_node;
                if (point.size() >= 3) {
                    z_new = point.at(2);
                    z_node = (*currentNode)->point.at(2);
                    splitModulo = 3;
                }
                // split on the first feature at even depth
                if (treeDepth % splitModulo == 0) {
                    // traverse the tree based on the feature split
                    // inserted feature smaller -> go left, else right
                    currentNode = x_new <= x_node ? &((*currentNode)->left) : &((*currentNode)->right);
                } else if (treeDepth % splitModulo == 1) {
                    currentNode = y_new <= y_node ? &((*currentNode)->left) : &((*currentNode)->right);
                } else if (treeDepth % splitModulo == 2) {
                    currentNode = z_new <= z_node ? &((*currentNode)->left) : &((*currentNode)->right);
                }
                treeDepth++;
            }
        }

    }

    // checks if comparisonNode is within the bounding box of target which is specified by distanceTol
    // returns true if it is in bounding box, else false
    bool _checkIfInBox(const std::vector<float>& target, float distanceTol, Node* comparisonNode) const {
        // 2D case
        bool inX = target.at(0) + distanceTol >= comparisonNode->point.at(0) &&
                   target.at(0) - distanceTol <= comparisonNode->point.at(0);
        bool inY = target.at(1) + distanceTol >= comparisonNode->point.at(1) &&
                   target.at(1) - distanceTol <= comparisonNode->point.at(1);

        if (target.size() == 2) {
            return (inX && inY);
        } else if (target.size() >= 3) {
            bool inZ = target.at(2) + distanceTol >= comparisonNode->point.at(2) &&
                       target.at(2) - distanceTol <= comparisonNode->point.at(2);
            return (inX && inY && inZ);
        }
    }

    // calculates the distance between the target and the current node, in 2d or 3d
    float _calculateDistance(const std::vector<float>& target, Node* comparisonNode) const {
        float xDistSquare = pow(comparisonNode->point.at(0) - target.at(0), 2);
        float yDistSquare = pow(comparisonNode->point.at(1) - target.at(1), 2);
        float zDistSquare = 0;
        if (target.size() >= 3) {
            zDistSquare = pow(comparisonNode->point.at(2) - target.at(2), 2);
        }
        return sqrt(xDistSquare + yDistSquare + zDistSquare);
    }

    void
    _searchRecurse(const std::vector<float>& target, float distanceTol, std::vector<int>& ids, Node* currentNode,
                   int depth) const {

        if (currentNode != nullptr) {
            // check if Node lies within target box
            if (_checkIfInBox(target, distanceTol, currentNode)) {
                // circular distance
                float distance = _calculateDistance(target, currentNode);
                // add node if close enough
                if (distance <= distanceTol) {
                    ids.push_back(currentNode->id);
                }
            }
            // get feature values on which to split while traversing the tree
            int splitModulo = 2;
            // adadpt feature split to 3D if necessary, just like in insert
            if (target.size() >= 3) {
                splitModulo = 3;
            }
            float targetVal = target.at(depth % splitModulo);
            float nodeVal = currentNode->point.at(depth % splitModulo);
            // check and explore tree branch
            if (targetVal - distanceTol < nodeVal) {
                _searchRecurse(target, distanceTol, ids, currentNode->left, depth + 1);
            }
            if (targetVal + distanceTol >= nodeVal) {
                _searchRecurse(target, distanceTol, ids, currentNode->right, depth + 1);
            }
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(const std::vector<float> target, float distanceTol) {
        std::vector<int> ids;

        _searchRecurse(target, distanceTol, ids, root, 0);

        return ids;
    }
};




