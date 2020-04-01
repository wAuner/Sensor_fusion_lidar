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
                float x_new = point.at(0);
                float y_new = point.at(1);
                float x_node = (*currentNode)->point.at(0);
                float y_node = (*currentNode)->point.at(1);
                // split on the first feature at even depth
                if (treeDepth % 2 == 0) {
                    // traverse the tree based on the feature split
                    // inserted feature smaller -> go left, else right
                    currentNode = x_new <= x_node ? &((*currentNode)->left) : &((*currentNode)->right);
                } else {
                    currentNode = y_new <= y_node ? &((*currentNode)->left) : &((*currentNode)->right);
                }
                treeDepth++;
            }
        }

    }

    void
    searchRecurse(const std::vector<float>& target, float distanceTol, std::vector<int>& ids, Node* currentNode,
                  int depth) const {

        if (currentNode != nullptr) {
            // check if Node lies within target box
            if ((target.at(0) + distanceTol >= currentNode->point.at(0) &&
                 target.at(0) - distanceTol <= currentNode->point.at(0))
                && (target.at(1) + distanceTol >= currentNode->point.at(1) &&
                    target.at(1) - distanceTol <= currentNode->point.at(1))) {
                // circular distance
                float distance = sqrt(pow(currentNode->point.at(0) - target.at(0), 2) +
                                      pow(currentNode->point.at(1) - target.at(1), 2));
                // add node if close enough
                if (distance <= distanceTol) {
                    ids.push_back(currentNode->id);
                }
            }
            // get feature values on which to split while traversing the tree
            float targetVal = target.at(depth % 2);
            float nodeVal = currentNode->point.at(depth % 2);
            // check and explore tree branch
            if (targetVal - distanceTol < nodeVal) {
                searchRecurse(target, distanceTol, ids, currentNode->left, depth+1);
            }
            if (targetVal + distanceTol >= nodeVal) {
                searchRecurse(target, distanceTol, ids, currentNode->right, depth+1);
            }
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(const std::vector<float> target, float distanceTol) {
        std::vector<int> ids;

        searchRecurse(target, distanceTol, ids, root, 0);

        return ids;
    }
};




