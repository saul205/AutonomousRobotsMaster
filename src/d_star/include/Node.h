#ifndef Node_H
#define Node_H

#include <string>
#include <vector>
#include <geometry_msgs/PoseStamped.h>

class Node
{
    private:
        std::vector <int> point;
        std::pair<Node *, int> parent;
        std::vector<std::pair<Node*, int>> neighbours;

        int countNodesRec(Node *root, int& count);

    public:
        Node();
		Node(std::vector <int> point_);
		~Node();
		bool hasNeighbours();
        void addNeighbour(Node* neighbourt, int cost);
        void appendNeighbour(Node *neighbour, int cost);
        void setParent(Node *parent, int cost);

        bool hasNeighbours() const { return neighbours.size() > 0; }
        bool hasParent();

        std::pair<Node*, int> getParent();
        std::vector<std::pair<Node*, int>> Node::getNeighbours();

		std::vector <int> getNode();
        void setNode(int x, int y);
		void printNode();
		void printTree();

		std::vector <std::vector <int>> returnSolution();
};

#endif
