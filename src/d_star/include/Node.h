#ifndef Node_H
#define Node_H

#include <string>
#include <vector>
#include <geometry_msgs/PoseStamped.h>

enum TAGS{
    OPEN,
    CLOSED,
    NEW
};

class Node
{
    private:
        std::vector <int> point;
        int parent_idx;
        std::vector<std::pair<Node*, float>> neighbours;

        int countNodesRec(Node *root, int& count);

    public:

        float k, h;
        TAGS tag;

        Node();
		Node(std::vector <int> point_);
		~Node();
		bool hasNeighbours();
        void addNeighbour(Node* neighbour, float cost);
        void appendNeighbour(Node *neighbour, float cost);
        void modifyNeighbourCost(int idx, float cost);
        void setParent(Node *parent);
        void setParent(int index);

        bool hasNeighbours() const { return neighbours.size() > 0; }
        bool hasParent();

        std::pair<Node*, float> getParent();
        std::vector<std::pair<Node*, float>> getNeighbours();

		std::vector <int> getNode();
        void setNode(int x, int y);
		void printNode();
		void printTree();

		std::vector <std::vector <int>> returnSolution();
};

#endif
